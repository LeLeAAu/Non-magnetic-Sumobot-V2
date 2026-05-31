// Chạy trên Core 0
// Thu thập raw data, lọc nhiễu, tính toán vị trí
#include "SensorTask.h"
#include "Config.h"
#include <Wire.h>
#include <VL53L1X.h>
#include <SparkFunLSM6DS3.h>
#include <math.h>    
#include <atomic>
#include "telemetry_utils.h"

#if SIMULATION_MODE
    static float sim_angle = 0.0f;
    static float sim_dist_front = 500.0f;
    static float sim_v_e = 0.0f;
    static uint32_t last_sim_time = 0;
    static int sim_step = 0;
#endif


static uint8_t tx_buffer[256];
static uint8_t cobs_buffer[256];

// Lịch sử cảm biến
static uint16_t dist_history[5][3];
static uint8_t dist_idx[5] = {0, 0, 0, 0, 0};

// HÀM PHỤ TRỢ
// Chuyển đổi PWM sang vận tốc tuyến tính xấp xỉ mm/s
float getEstimatedVelocity(int pwm) {
    // Tránh việc tính toán khi pwm = 0
    if (pwm < 30) return 0; 
    return ((float)pwm / 150.0) * V_MAX_60;
}

// Hàm tính Median siêu tốc cho 3 phần tử
// Chỉ tốn tối đa 3 phép so sánh thay vì dùng vòng lặp for/while
uint16_t getMedian(uint16_t* history_array) {
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
    static uint32_t last_yaw_time = millis();
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
#if !SIMULATION_MODE
        static float real_v_e = 0.0f;
        static bool real_closingFast = false;
        static float real_t_robot = 0.0f;
        static float real_t_enemy = 9999.0f;

        // TOÀN BỘ LOGIC CẢM BIẾN THẬT
        bool has_new_tof = false;
        
        // ĐỌC CẢM BIẾN VÀ CƠ CHẾ WATCHDOG
        for (int i = 0; i < 5; i++) {
            if (sensorsToF[i].dataReady()) { // Chỉ đọc khi ToF đã báo đo xong - non-blocking mode
                int raw_dist_calc = (int)sensorsToF[i].read(false);
                
                // Kiểm tra trạng thái dữ liệu (RangeStatus == 0 là đo thành công hợp lệ)
                // Nếu báo lỗi (ngoài tầm, nhiễu sáng, yếu...), ép thẳng khoảng cách thành 8190
                if (sensorsToF[i].ranging_data.range_status != 0) {
                    raw_dist_calc = 8190;
                } else {
                    raw_dist_calc += TOF_OFFSET[i];
                }
                
                uint16_t raw_dist = (raw_dist_calc < 0) ? 0 : (uint16_t)raw_dist_calc;
                // Trạng thái idle/chờ -> dùng bộ lọc median để lấy mốc tĩnh
                if (currentState == STATE_IDLE || currentState == STATE_INIT_DELAY) {
                    dist_history[i][dist_idx[i]] = raw_dist;
                    dist_idx[i] = (dist_idx[i] + 1) % MEDIAN_WINDOW;
                    tempData.dist[i] = getMedian(dist_history[i]);
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
                real_v_e = (V_EMA_ALPHA * v_raw) + ((1.0 - V_EMA_ALPHA) * real_v_e);

                // Phân loại nhịp độ trận đấu
                if (real_v_e > 450.0) real_closingFast = true; // Địch đang lao nhanh
                else if (real_v_e < 350.0) real_closingFast = false;
            } else {
                real_v_e = 0.0;
                real_closingFast = false;
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
            
            real_t_robot = (fabsf(theta_target) / OMEGA_60) + (l_path / V_MAX_60);
            real_t_enemy = (real_v_e < 5.0) ? 9999.0 : (R_SIDE / real_v_e);
            if (d <= DIST_CLOSE || tempData.isTargetLost) {
                condition_flank_met = false; // Quá gần, không kịp tạt nữa
            } else { // Mếu đến được điểm sườn nhanh hơn địch lao tới ngã 3 + khoảng T_MARGIN an toàn
                condition_flank_met = ((real_t_robot + T_MARGIN) < real_t_enemy);
            }

            prev_d = d;
            last_kinematic_time = current_time;
        }
    } 
    else if (current_time - last_kinematic_time > 150) { 
        // Reset Kinematics nếu ToF mù quá lâu
        real_v_e = 0.0;
        real_closingFast = false;
        tempData.isTargetLost = true;
    }

    // Đẩy static data vào tempData (cho buffer hiện tại)
    tempData.v_e = real_v_e;
    tempData.closingFast = real_closingFast;
    tempData.t_robot = real_t_robot;
    tempData.t_enemy = real_t_enemy;
    tempData.v_0 = getEstimatedVelocity(tempData.current_PWM);

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
        
        if (tempData.accelX != 0.0f || tempData.accelY != 0.0f || tempData.accelZ != 0.0f) {
            last_imu_time = current_time;
        }

        // Nếu quá 200ms không có tín hiệu sống -> Ép khởi tạo lại
        if (current_time - last_imu_time > 200) {
            DEBUG_PRINTLN(">>> SENSOR HEALTH: IMU TIMEOUT/HUNG! Đang khởi tạo lại...");
            
            // Thử gọi lại begin(). Lưu ý: hàm này của thư viện SparkFun trả về 0 nếu thành công
            if (myIMU.begin() == 0) {
                DEBUG_PRINTLN(">>> IMU Revived thành công!");
                last_imu_time = current_time; // Gia hạn mạng sống để không bị loop khởi tạo
            } else {
                DEBUG_PRINTLN(">>> IMU Re-init THẤT BẠI!");
                // Ép giá trị gia tốc về 0 để FSM biết cảm biến đang mù tạm thời
                tempData.accelX = 0.0f;
                tempData.accelY = 0.0f;
                tempData.accelZ = 0.0f;
            }
        }

        // Tính góc nghiêng Euler từ Vector gia tốc trọng trường G
        tempData.pitch = atan2(-tempData.accelX, sqrt(tempData.accelY * tempData.accelY + tempData.accelZ * tempData.accelZ)) * 180.0 / M_PI;
        tempData.roll  = atan2(tempData.accelY, tempData.accelZ) * 180.0 / M_PI;

        // Tích phân Gyro Z để tính Yaw
        float dt_yaw = (current_time - last_yaw_time) / 1000.0f;
        last_yaw_time = current_time;
        float gyroZ = myIMU.readFloatGyroZ();
        if (fabsf(gyroZ) > 1.0f) { // Lọc nhiễu nhỏ (deadband)
            tempData.yaw += gyroZ * dt_yaw;
        }

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
        // Ràng buộc vật lý: Chỉ đang bế địch nếu TCRT bị che VÀ d0 phải sát mặt (< DIST_CLOSE) hoặc mù/chỉ lên trời (> DIST_BLIND)
        bool is_tcrt_triggered = (tcrt_detect_val <= TCRT_LIFT_TH);
        bool is_d0_valid = (tempData.dist[0] < DIST_CLOSE || tempData.dist[0] > DIST_BLIND);
        tempData.liftDetected = is_tcrt_triggered && is_d0_valid;
        
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
        // THUẬT TOÁN NHẬN DIỆN VA CHẠM (IMPACT DETECTION
        // 1. Chỉ lấy Vector 2D (Mặt phẳng XY), loại bỏ hoàn toàn nhiễu từ trọng lực trục Z
        float horizontal_accel = sqrt(tempData.accelX * tempData.accelX + tempData.accelY * tempData.accelY);
        
        // 2. Dùng bộ lọc EMA (Exponential Moving Average) để tạo "Gia tốc nền"
        // Gia tốc nền sẽ học theo chuyển động hiện tại của xe nhưng phản ứng chậm hơn
        static float ema_horizontal_accel = 0.0f;
        ema_horizontal_accel = (0.15f * horizontal_accel) + (0.85f * ema_horizontal_accel);
        
        // 3. Xung lực va chạm (Jolt) là sự chênh lệch ĐỘT NGỘT giữa gia tốc tức thời và gia tốc nền
        float impact_jolt = fabsf(horizontal_accel - ema_horizontal_accel);
        
        // 4. Kích hoạt cờ nếu giật cục vượt ngưỡng
        tempData.impactDetected = (impact_jolt > ACC_IMPACT_TH);

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

        // Ngắt mềm (Giữ nguyên lệnh Notify để đánh thức FSM)
        // Nếu phát hiện các event nguy hiểm -> không đợi FSM ở Core 1 tự quay lại loop -> Core 0 phát tín hiệu TaskNotify vào thẳng FSM ép nó wake up xử lí ngay trong 0ms
        if (tempData.edgeDetect || tempData.fallOut || tempData.beingLifted || tempData.impactDetected) {
            if (TaskFSMHandle != NULL) xTaskNotifyGive(TaskFSMHandle);
        }

        if (current_time - last_health_check >= 1000) {
            bool is_error_now = false;

            // Kiểm tra ToF Timeout & Hard Reset
            bool tof_timeout[5] = {false, false, false, false, false};
            bool need_tof_reset = false;

            for (int i = 0; i < 5; i++) {
                if (current_time - last_tof_update[i] > 1000) {
                    tof_timeout[i] = true;
                    need_tof_reset = true;
                    is_error_now = true;
                    DEBUG_PRINTF(">>> SENSOR HEALTH: ToF %d TIMEOUT! Flagged for reset...\n", i);
                    
                    // Kéo LOW tất cả các ToF bị lỗi CÙNG LÚC để tắt chúng đi
                    // Tránh việc nhiều ToF cùng boot lên và tranh chấp địa chỉ I2C mặc định 0x29
                    digitalWrite(XSHUT_PINS[i], LOW);
                }
            }

            if (need_tof_reset) {
                vTaskDelay(pdMS_TO_TICKS(50)); // Chờ tắt hẳn

                for (int i = 0; i < 5; i++) {
                    if (tof_timeout[i]) {
                        // Kéo HIGH TỪNG ToF MỘT
                        digitalWrite(XSHUT_PINS[i], HIGH);
                        vTaskDelay(pdMS_TO_TICKS(15)); // Khuyến cáo của datasheet là chờ ít nhất 1.2ms
                        
                        // Cố gắng khởi tạo lại và đổi địa chỉ
                        if (sensorsToF[i].init()) {
                            sensorsToF[i].setAddress(VLX_ADDRESSES[i]); // Đợi ToF này đổi địa chỉ an toàn xong
                            sensorsToF[i].setDistanceMode(VL53L1X::Long);
                            sensorsToF[i].setMeasurementTimingBudget(33000);
                            sensorsToF[i].startContinuous(34);
                            
                            DEBUG_PRINTF(">>> ToF %d Revived!\n", i);
                            last_tof_update[i] = current_time; // Gia hạn mạng sống thêm 1s
                        } else {
                            DEBUG_PRINTF(">>> ToF %d Reset FAILED!\n", i);
                        }
                        // Vòng lặp mới chuyển sang ToF tiếp theo để kéo HIGH
                    }
                }
            }

            // Kiểm tra IMU Disconnect
            if (tempData.accelX == 0.0f && tempData.accelY == 0.0f && tempData.accelZ == 0.0f) {
                is_error_now = true;
                DEBUG_PRINTLN(">>> SENSOR HEALTH: IMU DEAD/DISCONNECTED! (Health Check)");
                // Ở đây chỉ ghi nhận lỗi để nếu > 3 lần (3s), FSM sẽ đưa xe vào STATE_DEF_LAST_STAND.
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
#else
        // SIMULATION MODE: Đọc dữ liệu inject từ Python UI
        tempData.timestamp = current_time;

        if (current_time - last_sim_time >= 40) { 
            // Tự động tính vận tốc v_e (mm/s) dựa trên tốc độ bạn kéo slider d0
            static float prev_sim_d0 = 2000.0f;
            float dt = (current_time - last_sim_time) / 1000.0f;
            float v_raw = -(sim_in_dist[0] - prev_sim_d0) / dt; 
            
            // Lọc nhiễu nhẹ để số v_e khỏi nhảy loạn xạ khi tay bạn kéo slider giật cục
            sim_v_e = (0.3 * v_raw) + (0.7 * sim_v_e);
            prev_sim_d0 = sim_in_dist[0];
            
            last_sim_time = current_time;
        }
        
        tempData.v_e = sim_v_e;

        // Bơm data từ biến toàn cục vào buffer của xe
        tempData.dist[0] = sim_in_dist[0];
        tempData.dist[1] = sim_in_dist[1]; 
        tempData.dist[2] = sim_in_dist[2]; 
        tempData.dist[3] = sim_in_dist[3]; 
        tempData.dist[4] = sim_in_dist[4];

        tempData.line[0] = sim_in_line[0]; // FL
        tempData.line[1] = sim_in_line[1]; // FR
        tempData.line[2] = sim_in_line[2]; // BL
        tempData.line[3] = sim_in_line[3]; // BR
        uint16_t tcrt_detect_val = sim_in_line[4];

        // Logic giả lập mồi cho FSM hoạt động
        tempData.isTargetLost = (sim_in_dist[0] >= 2000 && sim_in_dist[1] >= 2000 && sim_in_dist[2] >= 2000 && sim_in_dist[3] >= 2000 && sim_in_dist[4] >= 2000);
        
        // Mô phỏng góc tạt ngang (Kéo d1, d2, d3, d4 để bẻ góc)
        float sum_x = 0.0, sum_y = 0.0, sum_weights = 0.0;
        
        for (int i = 0; i < 5; i++) {
            // Chỉ tính toán các cảm biến quét trúng mục tiêu trong tầm CONF_ENY
            if (tempData.dist[i] < CONF_ENY) {
                float d_val = (tempData.dist[i] < 1.0f) ? 1.0f : (float)tempData.dist[i];
                // Vật càng gần, trọng số càng lớn (tỷ lệ nghịch với bình phương khoảng cách)
                float weight = 1000000.0f / (d_val * d_val);
                sum_x += SENSOR_SIN[i] * weight;
                sum_y += SENSOR_COS[i] * weight;
                sum_weights += weight;
            }
        }

        if (sum_weights > 0.0) {
            tempData.enemy_angle = atan2(sum_x, sum_y) * 180.0 / M_PI; 
        } else {
            // Nếu không có cảm biến nào < CONF_ENY, giữ nguyên góc hoặc gán 0
            tempData.enemy_angle = 0.0f; 
        }

        tempData.closingFast = (tempData.v_e > 450.0f);
        tempData.v_0 = getEstimatedVelocity(tempData.current_PWM);

        // Giả lập IMU luôn thăng bằng
        tempData.pitch = 0.0f; tempData.roll = 0.0f;
        tempData.accelX = 0.0f; tempData.accelY = 0.0f; tempData.accelZ = -1.0f;

        // Bơm cờ hệ thống theo giá trị TCRT bạn cấu hình
        tempData.hardwareFailure = false;
        tempData.edgeDetect = (tempData.line[0] < TCRT_EDGE_TH || tempData.line[1] < TCRT_EDGE_TH || tempData.line[2] < TCRT_EDGE_TH || tempData.line[3] < TCRT_EDGE_TH);
        
        bool is_tcrt_triggered = (tcrt_detect_val <= TCRT_LIFT_TH);
        bool is_d0_valid = (tempData.dist[0] < DIST_CLOSE || tempData.dist[0] > DIST_BLIND);
        tempData.liftDetected = is_tcrt_triggered && is_d0_valid;
        
        // Đạp vạch trắng thì đánh thức FSM ngay lập tức (Ngắt mô phỏng)
        if (tempData.edgeDetect) {
            xTaskNotifyGive(TaskFSMHandle); 
        }

        tempData.fallOut = false;
        tempData.impactDetected = false;
        tempData.sideDanger = false;
        tempData.liftedFront = false;
        tempData.liftedRear = false;
        tempData.beingLifted = false;
#endif
        sysBuffer[write_index] = tempData;
        read_index.store(write_index);

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


