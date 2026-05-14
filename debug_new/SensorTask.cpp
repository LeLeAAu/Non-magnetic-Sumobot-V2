#include "Config.h"
#include "MotorControl.h"
#include "DisplayFace.h"
#include "SensorTask.h"
#include "FSMTask.h"

float getEstimatedVelocity(int pwm) {
    // Tránh việc tính toán khi pwm = 0
    if (pwm < 30) return 0; 
    return ((float)pwm / 150.0) * V_MAX_60;
}

// Hàm tính Median siêu tốc chuyên dụng cho mảng 3 phần tử
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

void TaskSensorCode(void * pvParameters) {
    static uint32_t flk_timer_start = 0;
    const float T_MARGIN = 0.1; 
    static uint32_t last_imu_time = millis();
    static uint16_t last_valid_dist[5] = {8190, 8190, 8190, 8190, 8190};
    static uint8_t spike_count[5] = {0, 0, 0, 0, 0};
    static bool condition_flank_met = false;
    static uint32_t last_kinematic_time = 0;

    // [THÊM MỚI] - Mảng lưu thời gian cuối cùng ToF trả dữ liệu
    static uint32_t last_tof_update[5] = {0, 0, 0, 0, 0};

    for(;;) {
        uint32_t current_time = millis();
        SystemData tempData;
        xSemaphoreTake(dataMutex, portMAX_DELAY);
        tempData = sysData;
        xSemaphoreGive(dataMutex);

        bool has_new_tof = false;
        
        // 1. ĐỌC CẢM BIẾN VÀ CHÓT WATCHDOG
        for (int i = 0; i < 5; i++) {
            if (sensorsToF[i].dataReady()) {
                uint16_t raw_dist = sensorsToF[i].read(false);
                if (currentState == STATE_IDLE || currentState == STATE_INIT_DELAY) {
                    dist_history[i][dist_idx[i]] = raw_dist;
                    dist_idx[i] = (dist_idx[i] + 1) % MEDIAN_WINDOW;
                    tempData.dist[i] = getMedian(dist_history[i], MEDIAN_WINDOW);
                    last_valid_dist[i] = tempData.dist[i];
                } else {
                    int delta_dist = (int)raw_dist - (int)last_valid_dist[i];
                    if (delta_dist > 400 && last_valid_dist[i] < 1500) { 
                        spike_count[i]++;
                        if (spike_count[i] <= 2) raw_dist = last_valid_dist[i];
                        else last_valid_dist[i] = raw_dist;
                    } else {
                        spike_count[i] = 0;
                        last_valid_dist[i] = raw_dist;
                    }
                    tempData.dist[i] = raw_dist;
                }
                
                // [THÊM MỚI] - Cập nhật lại nhịp tim cho con ToF này
                last_tof_update[i] = current_time; 
                has_new_tof = true;
                
            } else {
                // [THÊM MỚI] - WATCHDOG: Rà soát xem con ToF này có bị treo không
                if (last_tof_update[i] != 0 && (current_time - last_tof_update[i] > 200)) {
                    // Nếu quá 200ms không có data (và không phải lúc mới boot)
                    // -> Ép nó thành 8190 (mù tịt) để tránh tạo bóng ma
                    tempData.dist[i] = 8190;
                }
            }
        }

        // 2. TÌM VỊ TRÍ VÀ KHOẢNG CÁCH CHUNG (BẤT CỨ KHI NÀO CÓ DATA MỚI)
        if (has_new_tof) {
            float sum_x = 0.0, sum_y = 0.0, sum_weights = 0.0;
            float d_closest = 8190.0;

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
                tempData.isTargetLost = false;
                tempData.enemy_angle = atan2(sum_x, sum_y) * 180.0 / M_PI; 
                d = d_closest; 
            } else {
                tempData.isTargetLost = true;
                d = 8190.0;
            }

            // 3. TÍNH VẬN TỐC THEO CHU KỲ CỐ ĐỊNH (Khử nhiễu bất đồng bộ)
            float dt_kinematic = (current_time - last_kinematic_time) / 1000.0;

            if (dt_kinematic >= 0.035) { 
                static float prev_d = 8190.0;
                
                if (!tempData.isTargetLost && prev_d < 8190.0) {
                    float delta_d = d - prev_d;
                    
                    if (fabsf(delta_d) <= V_DEADBAND_MM) delta_d = 0.0;

                    float v_raw = -delta_d / dt_kinematic;
                    tempData.v_e = (V_EMA_ALPHA * v_raw) + ((1.0 - V_EMA_ALPHA) * tempData.v_e);
                    tempData.v_0 = getEstimatedVelocity(tempData.current_PWM);

                    if (tempData.v_e > 450.0) tempData.closingFast = true;
                    else if (tempData.v_e < 350.0) tempData.closingFast = false;
                } else {
                    tempData.v_e = 0.0;
                    tempData.closingFast = false;
                }
                
                float alpha = tempData.enemy_angle;
                float x_e = d * sin(alpha * M_PI / 180.0);
                float y_e = d * cos(alpha * M_PI / 180.0);
                float x_p = x_e + R_SIDE * sin((alpha - 90.0) * M_PI / 180.0);
                float y_p = y_e + R_SIDE * cos((alpha - 90.0) * M_PI / 180.0);
                float l_path = sqrt(x_p * x_p + y_p * y_p);
                float theta_target = atan2(x_p, y_p) * 180.0 / M_PI;
                
                tempData.t_robot = (fabsf(theta_target) / OMEGA_60) + (l_path / V_MAX_60);
                tempData.t_enemy = (tempData.v_e < 5.0) ? 9999.0 : (R_SIDE / tempData.v_e);
                if (d <= DIST_CLOSE || tempData.isTargetLost) {
                    condition_flank_met = false;
                } else {
                    condition_flank_met = ((tempData.t_robot + T_MARGIN) < tempData.t_enemy);
                }

                prev_d = d;
                last_kinematic_time = current_time;
            }
        } 
        else if (current_time - last_kinematic_time > 150) { 
            tempData.v_e = 0.0;
            tempData.closingFast = false;
            tempData.isTargetLost = true;
        }

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

        // --- (CÁC PHẦN CẢM BIẾN TCRT, IMU BÊN DƯỚI GIỮ NGUYÊN) ---
        tempData.line[0] = analogRead(PIN_TCRT_FL);
        tempData.line[1] = analogRead(PIN_TCRT_FR);
        tempData.line[2] = analogRead(PIN_TCRT_BL);
        tempData.line[3] = analogRead(PIN_TCRT_BR);
        uint16_t tcrt_detect_val = analogRead(PIN_TCRT_DETECT);

        tempData.accelX = myIMU.readFloatAccelX();
        tempData.accelY = myIMU.readFloatAccelY();
        tempData.accelZ = -myIMU.readFloatAccelZ();
        
        tempData.pitch = atan2(-tempData.accelX, sqrt(tempData.accelY * tempData.accelY + tempData.accelZ * tempData.accelZ)) * 180.0 / M_PI;
        tempData.roll  = atan2(tempData.accelY, tempData.accelZ) * 180.0 / M_PI;

        bool pitchUp   = (tempData.pitch > PITCH_TH);
        bool pitchDown = (tempData.pitch < -PITCH_TH); 
        bool rollChange = fabsf(tempData.roll) > PITCH_TH; 
        bool isTipping = pitchUp || pitchDown || rollChange;

        bool ignore_front = pitchUp;
        bool ignore_rear  = pitchDown;

        bool edge_FL = (!ignore_front) && (tempData.line[0] <= TCRT_EDGE_TH);
        bool edge_FR = (!ignore_front) && (tempData.line[1] <= TCRT_EDGE_TH);
        bool edge_BL = (!ignore_rear)  && (tempData.line[2] <= TCRT_EDGE_TH);
        bool edge_BR = (!ignore_rear)  && (tempData.line[3] <= TCRT_EDGE_TH);

        bool raw_edge = edge_FL || edge_FR || edge_BL || edge_BR;
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

        tempData.liftDetected = (tcrt_detect_val <= TCRT_LIFT_TH);
        bool enemy_at_rear = (tempData.dist[3] < WARN_DIST || tempData.dist[4] < WARN_DIST);

        tempData.liftedFront = pitchUp && (tempData.dist[0] < WARN_DIST) && (!tempData.liftDetected);
        tempData.liftedRear  = pitchDown && enemy_at_rear;
        tempData.fallOut = (!tempData.liftDetected) && isTipping;
        tempData.beingLifted = (!tempData.liftDetected) && pitchUp && (tempData.dist[0] < WARN_DIST);

        float current_a_mag = sqrt(tempData.accelX * tempData.accelX + tempData.accelY * tempData.accelY + tempData.accelZ * tempData.accelZ);
        static float prev_a_mag = 1.0; 
        float delta_a = fabsf(current_a_mag - prev_a_mag);
        prev_a_mag = current_a_mag;
        tempData.impactDetected = (delta_a > ACC_IMPACT_TH);

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

        xSemaphoreTake(dataMutex, portMAX_DELAY);
        sysData = tempData; 
        xSemaphoreGive(dataMutex);

        if (tempData.edgeDetect || tempData.fallOut || tempData.beingLifted || tempData.impactDetected) {
            if (TaskFSMHandle != NULL) xTaskNotifyGive(TaskFSMHandle);
        }
        vTaskDelay(5 / portTICK_PERIOD_MS);
    }
}

