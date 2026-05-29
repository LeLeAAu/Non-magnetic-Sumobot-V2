// Chạy trên Core 0
// Thu thập raw data, lọc nhiễu, tính toán vị trí
#include "SensorTask.h"
#include "Config.h"
#include <Wire.h>
#include <VL53L1X.h>
#include <SparkFunLSM6DS3.h>
#include <math.h>    
#include <atomic>
#include "COBS.h"
#include "crc16.h"


static uint8_t tx_buffer[256];
static uint8_t cobs_buffer[256];

// HÀM PHỤ TRỢ
// Chuyển đổi PWM sang vận tốc tuyến tính xấp xỉ mm/s
float getEstimatedVelocity(int pwm) {
    // Tránh việc tính toán khi pwm = 0
    if (pwm < 30) return 0; 
    return ((float)pwm / 150.0) * V_MAX_60;
}

// Hàm tính Median siêu tốc cho 3 phần tử
// Chỉ tốn tối đa 3 phép so sánh thay vì dùng vòng lặp for/while
uint16_t getMedian(uint16_t* history_array, uint8_t size) {
    uint16_t a = history_array[0];
    uint16_t b = history_array[1];
    uint16_t c = history_array[2];

    // Sorting network cho 3 biến (Chỉ tốn tối đa 3 phép so sánh)
    if (a > b) { uint16_t tmp = a; a = b; b = tmp; }
    if (b > c) { uint16_t tmp = b; b = c; c = tmp; }
    if (a > b) { uint16_t tmp = a; a = b; b = tmp; }

    // Phần tử ở giữa giờ chắc chắn nằm ở b
    return b;
}

// Luồng chính của core 0
void TaskSensorCode(void * pvParameters) {
    // Static const để lưu trữ trạng thái qua các chu kì lấy mẫu
    static uint32_t flk_timer_start = 0;
    const float T_MARGIN = 0.1; 
    static uint32_t last_imu_time = millis();
    static uint16_t last_valid_dist[5] = {8190, 8190, 8190, 8190, 8190};
    static uint8_t spike_count[5] = {0, 0, 0, 0, 0};
    static bool condition_flank_met = false;
    static uint32_t last_kinematic_time = 0;
    uint32_t last_health_check = millis();
    uint8_t sensor_error_count = 0;
    uint16_t last_tcrt_vals[5] = {0, 0, 0, 0, 0};

    // Mảng lưu thời gian cuối cùng ToF trả dữ liệu
    static uint32_t last_tof_update[5] = {0, 0, 0, 0, 0};

    for(;;) {
        uint32_t current_time = millis();

        // Xác định xem FSM đang đọc buffer nào, ta sẽ viết vào buffer còn lại
        uint8_t write_index = 1 - read_index.load(); 

        // Lấy lại dữ liệu cũ của chính buffer đó làm nền tảng tính toán tiếp
        SystemData tempData = sysBuffer[write_index]; 

        // Đọc trực tiếp giá trị PWM từ Core 1 gửi sang thông qua biến Atomic riêng biệt
        tempData.current_PWM = active_pwm.load();

        bool has_new_tof = false;
        
        // ĐỌC CẢM BIẾN VÀ CƠ CHẾ WATCHDOG
        for (int i = 0; i < 5; i++) {
            if (sensorsToF[i].dataReady()) { // Chỉ đọc khi ToF đã báo đo xong - non-blocking mode
                uint16_t raw_dist = sensorsToF[i].read(false);
                // Trạng thái idle/chờ -> dùng bộ lọc median để lấy mốc tĩnh
                if (currentState == STATE_IDLE || currentState == STATE_INIT_DELAY) {
                    dist_history[i][dist_idx[i]] = raw_dist;
                    dist_idx[i] = (dist_idx[i] + 1) % MEDIAN_WINDOW;
                    tempData.dist[i] = getMedian(dist_history[i], MEDIAN_WINDOW);
                    last_valid_dist[i] = tempData.dist[i];
                } else { // Các trạng thái khác cần tốc độ cao nên dùng Spike Filter (Lọc gai nhiễu)
                    int delta_dist = (int)raw_dist - (int)last_valid_dist[i];
                    // Nếu khoảng cách đột ngột thay đổi > 200mmm (Có thể do nhiễu bóng ma hoặc tay trọng tài)
                    if (delta_dist > 200 && last_valid_dist[i] < 1500) { 
                        spike_count[i]++;
                        // Bỏ qua lỗi đầu tiên. Nếu xuất hiện 2 lần liên tiếp -> cập nhật
                        if (spike_count[i] <= 1) raw_dist = last_valid_dist[i];
                        else last_valid_dist[i] = raw_dist;
                    } else {
                        spike_count[i] = 0;
                        last_valid_dist[i] = raw_dist;
                    }
                    tempData.dist[i] = raw_dist;
                }
                
                last_tof_update[i] = current_time; // Cập nhật _nhịp tim_ cho WatchDog
                has_new_tof = true;
                
            } else {
                // WATCHDOG: Rà soát I2C
                if (last_tof_update[i] != 0 && (current_time - last_tof_update[i] > 200)) {
                    // Nếu quá 200ms không có data (và không phải lúc mới boot)
                    // -> Ép nó thành 8190 (mù tịt) để tránh tạo bóng ma
                    tempData.dist[i] = 8190;
                }
            }
        }

        // TÍNH TOẠ ĐỘ TRỌNG TÂM ĐỊCH & KINEMATICS
        if (has_new_tof) {
            float sum_x = 0.0, sum_y = 0.0, sum_weights = 0.0;
            float d_closest = 8190.0;

            // Tính trung bình có trọng số dựa trên bảng Lookup Sin/Cos
            // Cảm biến bắt được địch ở gần hơn sẽ có trọng số - weight - cao hơn trong việc quyết định góc
            for (int i = 0; i < 5; i++) {
                if (tempData.dist[i] < CONF_ENY) {
                    float d_val = (tempData.dist[i] < 1.0f) ? 1.0f : (float)tempData.dist[i];
                    float weight = 1000000.0f / (d_val * d_val);
                    sum_x += SENSOR_SIN[i] * weight;
                    sum_y += SENSOR_COS[i] * weight;
                    sum_weights += weight;
                    if (tempData.dist[i] < d_closest) d_closest = tempData.dist[i];
                }
            }

            float d;
            if (sum_weights > 0.0) {
                tempData.isTargetLost = false; // Hàm atan2 tự động xử lí dấu và trả về góc từ -180 đến 180 độ
                tempData.enemy_angle = atan2(sum_x, sum_y) * 180.0 / M_PI; 
                d = d_closest; 
            } else {
                tempData.isTargetLost = true;
                d = 8190.0;
            }

            // TÍNH TOÁN ĐỘNG HỌC CHU KÌ ~40ms
            float dt_kinematic = (current_time - last_kinematic_time) / 1000.0;

            if (dt_kinematic >= 0.040) { 
                static float prev_d = 8190.0;
                
                if (!tempData.isTargetLost && prev_d < 8190.0) {
                    float delta_d = d - prev_d;
                    // Lọc Deadband: trừ khử nhiễu rung lắc
                    if (fabsf(delta_d) <= V_DEADBAND_MM) delta_d = 0.0;

                    float v_raw = -delta_d / dt_kinematic; // Vận tốc tương đối: Âm -> đang ra xa; Dương -> đang tiếp cận

                    // Bộ lọc Expotential Moving Average EMA
                    tempData.v_e = (V_EMA_ALPHA * v_raw) + ((1.0 - V_EMA_ALPHA) * tempData.v_e);
                    tempData.v_0 = getEstimatedVelocity(tempData.current_PWM);

                    // Phân loại nhịp độ trận đấu
                    if (tempData.v_e > 450.0) tempData.closingFast = true; // Địch đang lao nhanh
                    else if (tempData.v_e < 350.0) tempData.closingFast = false;
                } else {
                    tempData.v_e = 0.0;
                    tempData.closingFast = false;
                }
                
                // Tính toán Intercept Point để tạt sườn
                // Tính thời gian cần để chạy vòng qua sườn địch (t_robot) so với thời gian địch lao tới (t_enemy)
                float alpha = tempData.enemy_angle;
                float x_e = d * sin(alpha * M_PI / 180.0);
                float y_e = d * cos(alpha * M_PI / 180.0);
                // Tạo Projection Point bên hông địch
                float x_p = x_e + R_SIDE * sin((alpha - 90.0) * M_PI / 180.0);
                float y_p = y_e + R_SIDE * cos((alpha - 90.0) * M_PI / 180.0);
                float l_path = sqrt(x_p * x_p + y_p * y_p);
                float theta_target = atan2(x_p, y_p) * 180.0 / M_PI;
                
                tempData.t_robot = (fabsf(theta_target) / OMEGA_60) + (l_path / V_MAX_60);
                tempData.t_enemy = (tempData.v_e < 5.0) ? 9999.0 : (R_SIDE / tempData.v_e);
                if (d <= DIST_CLOSE || tempData.isTargetLost) {
                    condition_flank_met = false; // Quá gần, không kịp tạt nữa
                } else { // Mếu đến được điểm sườn nhanh hơn địch lao tới ngã 3 + khoảng T_MARGIN an toàn
                    condition_flank_met = ((tempData.t_robot + T_MARGIN) < tempData.t_enemy);
                }

                prev_d = d;
                last_kinematic_time = current_time;
            }
        } 
        else if (current_time - last_kinematic_time > 150) { 
            // Reset Kinematics nếu ToF mù quá lâu
            tempData.v_e = 0.0;
            tempData.closingFast = false;
            tempData.isTargetLost = true;
        }

        // DEBOUNCE CỜ TẠT SƯỜN - CHỐNG TÍN HIỆU CHẬP CHỜN
        static uint32_t last_flk_true_time = 0;
        if (condition_flank_met) {
            last_flk_true_time = current_time;
            if (flk_timer_start == 0) flk_timer_start = current_time;
            if ((current_time - flk_timer_start) >= FLK_STABLE_TIME) tempData.flkPossible = true;
        } else {
            if ((current_time - last_flk_true_time) > FLK_DEBOUNCE_TIME) {
                flk_timer_start = 0;
                tempData.flkPossible = false;
            }
        }

        // ĐỌC CẢM BIẾN IMU VÀ DÒ LINE TCRT
        tempData.line[0] = analogRead(PIN_TCRT_FL);
        tempData.line[1] = analogRead(PIN_TCRT_FR);
        tempData.line[2] = analogRead(PIN_TCRT_BL);
        tempData.line[3] = analogRead(PIN_TCRT_BR);
        uint16_t tcrt_detect_val = analogRead(PIN_TCRT_DETECT);

        tempData.accelX = myIMU.readFloatAccelX();
        tempData.accelY = myIMU.readFloatAccelY();
        tempData.accelZ = -myIMU.readFloatAccelZ();
        
        // Tính góc nghiêng Euler từ Vector gia tốc trọng trường G
        tempData.pitch = atan2(-tempData.accelX, sqrt(tempData.accelY * tempData.accelY + tempData.accelZ * tempData.accelZ)) * 180.0 / M_PI;
        tempData.roll  = atan2(tempData.accelY, tempData.accelZ) * 180.0 / M_PI;

        bool pitchUp   = (tempData.pitch > PITCH_TH); // Ngóc đầu lên
        bool pitchDown = (tempData.pitch < -PITCH_TH);  // Cắm đầu xuống
        bool rollChange = fabsf(tempData.roll) > PITCH_TH;  // Nghiêng lật xe
        bool isTipping = pitchUp || pitchDown || rollChange;

        // Khi xe bị hếch lên, mắt TCRT trước sẽ nhấc khỏi mặt đất, nhận sai thành vạch trắng thành vực sâu.
        // Phải dùng IMU để ignore cảnh báo giả này lại
        bool ignore_front = pitchUp;
        bool ignore_rear  = pitchDown;

        bool edge_FL = (!ignore_front) && (tempData.line[0] <= TCRT_EDGE_TH);
        bool edge_FR = (!ignore_front) && (tempData.line[1] <= TCRT_EDGE_TH);
        bool edge_BL = (!ignore_rear)  && (tempData.line[2] <= TCRT_EDGE_TH);
        bool edge_BR = (!ignore_rear)  && (tempData.line[3] <= TCRT_EDGE_TH);

        bool raw_edge = edge_FL || edge_FR || edge_BL || edge_BR;

        // Lọc nhiễu vạch trắng 20ms
        static uint32_t safe_timer_start = 0;
        if (raw_edge) {
            tempData.edgeDetect = true;
            safe_timer_start = current_time; 
        } else {
            if (current_time - safe_timer_start >= 20) tempData.edgeDetect = false;
            else tempData.edgeDetect = true;
        }

        static uint32_t last_edge_time = 0;
        if (tempData.edgeDetect) last_edge_time = current_time;

        // Phân tích trạng thái vật lí / tranh chấp
        tempData.liftDetected = (tcrt_detect_val <= TCRT_LIFT_TH);
        bool enemy_at_rear = (tempData.dist[3] < WARN_DIST || tempData.dist[4] < WARN_DIST);

        tempData.liftedFront = pitchUp && (tempData.dist[0] < WARN_DIST) && (!tempData.liftDetected);
        tempData.liftedRear  = pitchDown && enemy_at_rear;
        tempData.fallOut = (!tempData.liftDetected) && isTipping;
        tempData.beingLifted = (!tempData.liftDetected) && pitchUp && (tempData.dist[0] < WARN_DIST);

        // Phát hiện va chạm bằng cách lấy đạo hàm của độ lớn vector gia tốc
        float current_a_mag = sqrt(tempData.accelX * tempData.accelX + tempData.accelY * tempData.accelY + tempData.accelZ * tempData.accelZ);
        static float prev_a_mag = 1.0;  // Mặc định G = 1 khi đứng yên
        float delta_a = fabsf(current_a_mag - prev_a_mag);
        prev_a_mag = current_a_mag;
        tempData.impactDetected = (delta_a > ACC_IMPACT_TH); // Nếu gia tốc thay đổi đột ngột -> ngưỡng -> có va chạm

        // Cảnh báo địch rúc sườn
        static uint32_t side_danger_start = 0;
        bool isSideClose = (tempData.dist[1] < WARN_DIST || tempData.dist[2] < WARN_DIST || 
                            tempData.dist[3] < WARN_DIST || tempData.dist[4] < WARN_DIST);
        if (isSideClose) {
            if (side_danger_start == 0) side_danger_start = current_time;
            if (current_time - side_danger_start >= SIDE_DANGER_TIME) tempData.sideDanger = true;
        } else {
            side_danger_start = 0;
            tempData.sideDanger = false;
        }

        // Đóng dấu thời gian ngay trước khi xuất xưởng dữ liệu
        tempData.timestamp = millis(); 

        // Đẩy toàn bộ dữ liệu cục bộ vào buffer chuẩn bị hoán đổi
        sysBuffer[write_index] = tempData; 

        // Lật bài, kích hoạt buffer vừa ghi thành buffer cho FSM đọc
        read_index.store(write_index); 

        // Giữ nguyên lệnh Notify để đánh thức FSM ngay lập tức nếu có biến cố nguy hiểm
        if (tempData.edgeDetect || tempData.fallOut || tempData.beingLifted || tempData.impactDetected) {
            if (TaskFSMHandle != NULL) xTaskNotifyGive(TaskFSMHandle);
        }

        // Ngắt mềm
        // Nếu phát hiện các event nguy hiểm -> không đợi FSM ở Core 1 tự quay lại loop -> Core 0 phát tín hiệu TaskNotify vào thẳng FSM ép nó wake up xử lí ngay trong 0ms
        if (tempData.edgeDetect || tempData.fallOut || tempData.beingLifted || tempData.impactDetected) {
            if (TaskFSMHandle != NULL) xTaskNotifyGive(TaskFSMHandle);
        }

        if (current_time - last_health_check >= 1000) {
            bool is_error_now = false;

            // Kiểm tra ToF Timeout & Hard Reset
            for (int i = 0; i < 5; i++) {
                if (current_time - last_tof_update[i] > 1000) {
                    is_error_now = true;
                    DEBUG_PRINTF(">>> SENSOR HEALTH: ToF %d TIMEOUT! Hard Resetting...\n", i);
                    
                    // Ép reset cứng bằng chân XSHUT
                    digitalWrite(XSHUT_PINS[i], LOW);
                    vTaskDelay(pdMS_TO_TICKS(50));
                    
                    // Cấp nguồn lại và đợi IC khởi động
                    digitalWrite(XSHUT_PINS[i], HIGH);
                    vTaskDelay(pdMS_TO_TICKS(15)); // Khuyến cáo của datasheet là chờ ít nhất 1.2ms
                    
                    // Cố gắng khởi tạo lại (bỏ qua nếu I2C bus bị treo)
                    if (sensorsToF[i].init()) {
                        sensorsToF[i].setAddress(VLX_ADDRESSES[i]);
                        sensorsToF[i].setDistanceMode(VL53L1X::Long);
                        sensorsToF[i].setMeasurementTimingBudget(33000);
                        sensorsToF[i].startContinuous(34);
                        
                        DEBUG_PRINTF(">>> ToF %d Revived!\n", i);
                        last_tof_update[i] = current_time; // Gia hạn mạng sống thêm 1s
                    } else {
                        DEBUG_PRINTF(">>> ToF %d Reset FAILED!\n", i);
                    }
                }
            }

            // Kiểm tra IMU Disconnect
            // Gia tốc kế không bao giờ trả về 0.0 tuyệt đối ở cả 3 trục do luôn có trọng lực G và nhiễu vi cơ (MEMS noise).
            // Nếu cả 3 trục = 0.0 phẳng lì, nghĩa là bus I2C đã rớt hoặc IC treo.
            if (tempData.accelX == 0.0f && tempData.accelY == 0.0f && tempData.accelZ == 0.0f) {
                is_error_now = true;
                DEBUG_PRINTLN(">>> SENSOR HEALTH: IMU DEAD/DISCONNECTED!");
                // Gọi myIMU.begin() ở đây rất rủi ro vì nếu I2C treo phần cứng, nó sẽ block toàn bộ Core 0.
                // Tạm thời chỉ ghi nhận lỗi để FSM đưa xe vào trạng thái an toàn.
            }

            // Kiểm tra TCRT Stuck (Kẹt ADC)
            // ADC của ESP32 cực kỳ nhiễu. Nếu 5 chân đọc analog trả về MỘT GIÁ TRỊ Y HỆT KHÔNG ĐỔI trong suốt 1s,
            // 100% là ADC đã bị kẹt hoặc đứt dây nguồn (kẹt ở 0 hoặc 4095).
            if (tempData.line[0] == last_tcrt_vals[0] && tempData.line[1] == last_tcrt_vals[1] &&
                tempData.line[2] == last_tcrt_vals[2] && tempData.line[3] == last_tcrt_vals[3] &&
                tcrt_detect_val == last_tcrt_vals[4]) {
                is_error_now = true;
                DEBUG_PRINTLN(">>> SENSOR HEALTH: TCRT ADC STUCK!");
            }
            
            // Cập nhật mẫu ADC để đối chiếu chu kỳ sau
            last_tcrt_vals[0] = tempData.line[0];
            last_tcrt_vals[1] = tempData.line[1];
            last_tcrt_vals[2] = tempData.line[2];
            last_tcrt_vals[3] = tempData.line[3];
            last_tcrt_vals[4] = tcrt_detect_val;

            // Quyết định sinh tử
            if (is_error_now) {
                sensor_error_count++;
                if (sensor_error_count >= 3) { // Nếu lỗi > 3 lần (3 giây liên tiếp liệt)
                    tempData.hardwareFailure = true;
                    if (TaskFSMHandle != NULL) xTaskNotifyGive(TaskFSMHandle); // Đánh thức FSM ngay lập tức
                }
            } else {
                sensor_error_count = 0; 
                tempData.hardwareFailure = false;
            }

            last_health_check = current_time;
        }
        // Trả lại tài nguyên CPU
        vTaskDelay(5 / portTICK_PERIOD_MS);
    }
}


// Định nghĩa gói telemetry
struct __attribute__((packed)) TelemetryPacket {
    uint32_t timestamp;
    uint16_t dist[5];
    uint16_t line[4];
    float enemy_angle;
    float v_e;
    float v_0;
    float pitch;
    float roll;
    uint8_t state;
    uint8_t flags;
    uint8_t misc[2]; // padding
};

// Hàm gửi telemetry qua Serial
void send_telemetry(const SystemData &data, RobotState state) {
    TelemetryPacket pkt;
    // Khởi tạo struct và ép toàn bộ các byte (kể cả padding) về 0
    memset(&pkt, 0, sizeof(TelemetryPacket));
    pkt.timestamp = data.timestamp;
    memcpy(pkt.dist, data.dist, sizeof(pkt.dist));
    memcpy(pkt.line, data.line, sizeof(pkt.line));
    pkt.enemy_angle = data.enemy_angle;
    pkt.v_e = data.v_e;
    pkt.v_0 = data.v_0;
    pkt.pitch = data.pitch;
    pkt.roll = data.roll;
    pkt.state = (uint8_t)state;

    pkt.flags = (data.closingFast ? 0x01 : 0) |
                (data.flkPossible ? 0x02 : 0) |
                (data.fallOut ? 0x04 : 0) |
                (data.liftDetected ? 0x08 : 0) |
                (data.impactDetected ? 0x10 : 0) |
                (data.sideDanger ? 0x20 : 0) |
                (data.edgeDetect ? 0x40 : 0) |
                (data.isTargetLost ? 0x80 : 0);
    // misc có thể bỏ qua hoặc fill 0

    uint8_t *payload = (uint8_t*)&pkt;
    uint16_t payload_len = sizeof(TelemetryPacket);

    uint8_t frame[256];
    frame[0] = 0x01; // type
    frame[1] = payload_len & 0xFF;
    frame[2] = (payload_len >> 8) & 0xFF;
    memcpy(frame + 3, payload, payload_len);
    uint16_t crc = crc16_ccitt(frame, 3 + payload_len);
    frame[3 + payload_len] = crc & 0xFF;
    frame[3 + payload_len + 1] = (crc >> 8) & 0xFF;

    size_t frame_len = 3 + payload_len + 2;
    size_t cobs_len = cobs_encode(frame, frame_len, cobs_buffer);
    Serial.write(0x00);
    Serial.write(cobs_buffer, cobs_len);
    Serial.write(0x00);
}
