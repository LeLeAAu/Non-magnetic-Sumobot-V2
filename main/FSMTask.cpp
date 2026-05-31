// Xử lí dữ liệu từ Core 0 để đưa ra quyết định

#include "FSMTask.h"
#include "Config.h"
#include "MotorControl.h"
#include <Arduino.h>   
#include <math.h>     
#include <atomic>  

// Hàm phụ trợ nhằm tính toán góc xuất hiện nhiều nhất của đối thủ để dự đoán vị trí
float getModeAngle(float* history, int size);

void TaskFSMCode(void * pvParameters) {
   // Khởi tạo bộ nhớ chiễn thuật ĩnh
    static uint8_t failed_strike_count = 0;     // Đếm số lần ủi bế tắc
    static uint32_t target_lost_start_time = 0; // Theo dõi thời gian mất dấu
    // Biến cho việc build Histogram trong lúc chờ
    const int HIST_SIZE = 50;
    static float angle_histogram[HIST_SIZE];

    for (int i = 0; i < HIST_SIZE; i++) {
        angle_histogram[i] = 999.0; // 999.0 là giá trị rỗng
    }
    static int hist_idx = 0;
    static uint8_t stalemate_cycles = 0; // Đếm số lần bế tắc

    for(;;) {
        uint32_t fsm_current_time = millis();

        // Data spapshot
        // Đọc không khoá, lấy dữ liệu từ buffer mới nhất với tốc độ cao
        SystemData localData = sysBuffer[read_index.load()]; 
        // Chặn quyền điều khiển nếu đang bật CALIBR
#if CALIBRATION_MODE == 1
            if (currentState != STATE_CALIBRATION) {
                enterState(STATE_CALIBRATION);
            }
#endif

        // Giám sát watchdog: Kiểm tra xem dữ liệu có bị freeze không
        if (fsm_current_time - localData.timestamp > 150) { 
            DEBUG_PRINTLN(">>> CRITICAL ERROR: SENSOR CORE FROZEN! <<<");

            // Xử lý khẩn cấp khi bị mù thông tin (ví dụ: Phanh cứng hoặc Lùi thủ thế)
            if (currentState != STATE_IDLE && currentState != STATE_DEF_LAST_STAND) {
                driveBot(-PWM_MAX, -PWM_MAX); 
            }

            vTaskDelay(pdMS_TO_TICKS(5)); // Chờ nhẹ để nhường Core phục hồi bus I2C
            continue;                     // Bỏ qua chu kỳ logic FSM lỗi này
        }
        // TỪ ĐÂY TRỞ XUỐNG, CHỈ SỬ DỤNG localData. TUYỆT ĐỐI KHÔNG GỌI sysData

        // Cập nhật bộ đếm thời gian mất dấu
        if (localData.isTargetLost) {
            if (target_lost_start_time == 0) target_lost_start_time = fsm_current_time;
        } else {
            target_lost_start_time = 0;
        }
        uint32_t lost_duration = (target_lost_start_time > 0) ? (fsm_current_time - target_lost_start_time) : 0;

        // Global Safety layer
        bool is_self_jerk_blind_time = (fsm_current_time - state_start_time < 80);

        if (currentState != STATE_IDLE && currentState != STATE_INIT_DELAY) {

            // ƯU TIÊN 0: Cảm biến chết lâm sàng (> 3 lần)
            if (localData.hardwareFailure) {
                if (currentState != STATE_DEF_LAST_STAND) {
                    enterState(STATE_DEF_LAST_STAND);
                    DEBUG_PRINTLN(">>> CRITICAL ERROR: SENSORS DEAD -> HALTING/LAST STAND! <<<");
                }
                
                // GHI ĐÈ LỰC ĐỘNG CƠ: Thay vì để LAST_STAND dùng IMU hỏng để lật xe, ta phanh cứng nó lại để bảo vệ cơ khí.
                driveBot(0, 0); 
                
                // Xóa cờ Notification và bắt đầu lại chu kỳ
                ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5));
                continue; 
            }

            // ƯU TIÊN 1: Lật xe/rớt đài
            if (localData.fallOut) {
                if (currentState != STATE_DEF_LAST_STAND) {
                    enterState(STATE_DEF_LAST_STAND);
                    DEBUG_PRINTLN(">>> GLOBAL SAFETY: FALL OUT -> LAST STAND!");
                }
            }
            // ƯU TIÊN 2: Bị xới gầm / nhấc bổng
            else if (localData.liftedFront || localData.liftedRear || localData.beingLifted) {
                if (currentState != STATE_DEF_ANTI_LIFT) {
                    enterState(STATE_DEF_ANTI_LIFT);
                    DEBUG_PRINTLN(">>> GLOBAL SAFETY: BEING LIFTED -> ANTI_LIFT!");
                }
            }
            // ƯU TIÊN 3: Gặp Edge
            else if (localData.edgeDetect) {
                if (currentState != STATE_DEF_EDGE_AVOID && currentState != STATE_DEF_LAST_STAND) {
                    enterState(STATE_DEF_EDGE_AVOID);
                    DEBUG_PRINTLN(">>> GLOBAL SAFETY: EDGE DETECTED -> AVOID!");
                }
            }
            // ƯU TIÊN 4: Bị đâm mạnh
            else if (localData.impactDetected && !is_self_jerk_blind_time && 
                     currentState != STATE_ATK_STRIKE && 
                     currentState != STATE_ATK_LIFT &&
                     currentState != STATE_DEF_REAR_GUARD &&
                     currentState != STATE_DEF_SIDE_GUARD) {
                
                // Phân tích hướng bị đâm dựa trên việc ToF có đang bị mù hay không
                bool blind_hit = (localData.dist[0] > WARN_DIST && localData.dist[1] > WARN_DIST && 
                                  localData.dist[2] > WARN_DIST && localData.dist[3] > WARN_DIST && localData.dist[4] > WARN_DIST);
                bool side_hit = (localData.dist[3] < DIST_CLOSE || localData.dist[4] < DIST_CLOSE);

                if (blind_hit) {
                    enterState(STATE_DEF_REAR_GUARD); // Đâm từ điểm mù -> Thủ sau
                    DEBUG_PRINTLN(">>> GLOBAL SAFETY: BLIND IMPACT -> REAR GUARD!");
                } else if (side_hit && (currentState == STATE_SEARCH_ENEMY)) {
                    enterState(STATE_DEF_SIDE_GUARD); // Đâm ngang hông -> Thủ sườn
                    DEBUG_PRINTLN(">>> GLOBAL SAFETY: SIDE IMPACT -> SIDE GUARD!");
                } else if (localData.dist[0] < DIST_CLOSE || localData.dist[1] < DIST_CLOSE || localData.dist[2] < DIST_CLOSE) {
                    // Mặc định: Bị đâm từ phía trước -> TỔNG TẤN CÔNG!
                    enterState(STATE_ATK_STRIKE); 
                    DEBUG_PRINTLN(">>> GLOBAL SAFETY: FRONT IMPACT -> COUNTER STRIKE!");
                }
            }
        }

        // NORMAL FSM SWITCH-CASE
        switch (currentState) {
            
            // NHÓM 1: KHỞI TẠO (INIT)

            case STATE_CALIBRATION:
            {
                driveBot(0, 0); // Khóa cứng động cơ tuyệt đối
                
                // Bot không làm gì cả, chỉ ngồi ngoan ngoãn thu thập dữ liệu ToF, IMU, TCRT 
                // để Telemetry (Core 0) bắn lên Python UI cho bạn phân tích.
                break;
            }
            
            case STATE_IDLE:
            {
                driveBot(0, 0); // Khoá động cơ
                // Ghi nhớ vị trí địch liên tục vào histogram
                if (!localData.isTargetLost) {
                    angle_histogram[hist_idx] = localData.enemy_angle;
                    hist_idx = (hist_idx + 1) % HIST_SIZE;
                }
#if SIMULATION_MODE
                if (sim_in_ttp223 > 0) {
                    enterState(STATE_INIT_DELAY);
                    DEBUG_PRINTLN(">>> SIMULATION: TTP223 BUTTON CLICKED -> INIT_DELAY");
                }
#else
                if (digitalRead(PIN_TTP223) == HIGH) { 
                    enterState(STATE_INIT_DELAY); 
                    DEBUG_PRINTLN(">>> START: INIT_DELAY (3 seconds)");
                } 
#endif
                break;
            }

            case STATE_INIT_DELAY: 
            {
                driveBot(0, 0); // Khoá động cơ
                // Tiếp tục ghi nhớ góc địch trong lúc đếm ngược 3s
                if (!localData.isTargetLost) {
                    angle_histogram[hist_idx] = localData.enemy_angle;
                    hist_idx = (hist_idx + 1) % HIST_SIZE;
                } else {
                    // CẢ 5 MẮT ĐỀU THẤY TRỐNG TRƠN (> 2000mm) -> ĐỊCH Ở SAU LƯNG!
                    if (localData.dist[0] > DIST_BLIND && localData.dist[1] > DIST_BLIND && 
                        localData.dist[2] > DIST_BLIND && localData.dist[3] > DIST_BLIND && 
                        localData.dist[4] > DIST_BLIND) {
                        
                        angle_histogram[hist_idx] = 180.0; // Giả định địch ở góc 180
                        hist_idx = (hist_idx + 1) % HIST_SIZE;
                    }
                }

                if (fsm_current_time - state_start_time >= 0) { // Đã bỏ 3s đếm ngược theo yêu cầu -> Đưa ra quyết định ngay lập tức
                        float target_angle = getModeAngle(angle_histogram, HIST_SIZE); 
                        
                        if (localData.dist[0] < CONF_ENY) { // Địch trước mặt
                            enterState(STATE_ATK_LOCK); // Lock
                            DEBUG_PRINTLN(">>> DELAY XONG: ĐỊCH NGAY TRƯỚC MẶT -> LOCK!");
                        }
                        else if (localData.dist[1] < CONF_ENY || localData.dist[2] < CONF_ENY ||
                                localData.dist[3] < CONF_ENY || localData.dist[4] < CONF_ENY) {
                            enterState(STATE_ATK_LOCK);
                            DEBUG_PRINTLN(">>> DELAY XONG: ĐỊCH BÊN CẠNH");
                        }
                        else {
                            enterState(STATE_SEARCH_ENEMY);
                            float target_angle = getModeAngle(angle_histogram, HIST_SIZE);
                            if (target_angle != 999.0) { // Xoay robot về góc đã ghi nhớ
                                Serial.print(">>> DELAY XONG: TÌM KIẾM THEO GÓC GHI NHỚ: ");
                                DEBUG_PRINTLN(target_angle);
                            } else {
                                DEBUG_PRINTLN(">>> DELAY XONG: MÙ HOÀN TOÀN -> VÀO CHẾ ĐỘ QUÉT XOAY ỐC!");
                            }
                        }
                    }
                break;
            }

            // NHÓM 2: PHÒNG THỦ VÀ PHỤC HỒI
            case STATE_DEF_EDGE_AVOID:
            {
                int esc_l = 0;
                int esc_r = 0;
                // 1. LIÊN TỤC CẬP NHẬT TRẠNG THÁI SENSOR ĐỂ NÉ VẠCH
                bool ignore_front = (localData.pitch > PITCH_TH); 
                bool ignore_rear  = (localData.pitch < -PITCH_TH);

                bool edge_FL = (!ignore_front) && (localData.line[0] <= TCRT_EDGE_TH);
                bool edge_FR = (!ignore_front) && (localData.line[1] <= TCRT_EDGE_TH);
                bool edge_BL = (!ignore_rear)  && (localData.line[2] <= TCRT_EDGE_TH);
                bool edge_BR = (!ignore_rear)  && (localData.line[3] <= TCRT_EDGE_TH);

                if (localData.edgeDetect) {
                    // CẤP ĐỘ 1: MATADOR DODGE (Né đòn ủi thẳng mặt ở sát mép)
                    if (localData.dist[0] < WARN_DIST || localData.closingFast) {
                        if (edge_FL || edge_BL) { esc_l = -PWM_MED; esc_r = -PWM_MAX; } 
                        else { esc_l = -PWM_MAX; esc_r = -PWM_MED; }                    
                    } 
                    // CẤP ĐỘ 2: TẬN THẾ TIER (Giẫm 3-4 mắt)
                    else if ((edge_FL && edge_FR && edge_BL) || (edge_FL && edge_FR && edge_BR)) {
                        esc_l = -PWM_MAX; esc_r = -PWM_MAX;
                    }
                    else if ((edge_BL && edge_BR && edge_FL) || (edge_BL && edge_BR && edge_FR)) {
                        esc_l = PWM_MAX; esc_r = PWM_MAX;
                    }
                    // CẤP ĐỘ 3: THOÁT HIỂM CƠ BẢN
                    else {
                        if (edge_FL && edge_FR) { esc_l = -PWM_MAX; esc_r = -PWM_MAX; } 
                        else if (edge_BL && edge_BR) { esc_l = PWM_MAX; esc_r = PWM_MAX; } 
                        else if (edge_FL && edge_BR) { esc_l = -PWM_MAX; esc_r = -PWM_MED; } 
                        else if (edge_FR && edge_BL) { esc_l = -PWM_MED; esc_r = -PWM_MAX; } 
                        else if (edge_FL) { esc_l = -PWM_HIGH; esc_r = -PWM_MAX; } 
                        else if (edge_FR) { esc_l = -PWM_MAX; esc_r = -PWM_HIGH; } 
                        else if (edge_BL) { esc_l = PWM_HIGH; esc_r = PWM_MAX; }   
                        else if (edge_BR) { esc_l = PWM_MAX; esc_r = PWM_HIGH; }   
                        else { esc_l = -PWM_MAX; esc_r = -PWM_MAX; } 
                    }
                    
                    // Xuất lực ra động cơ né vạch
                    driveBot(esc_l, esc_r);
                }
                // 2. SẠCH VẠCH HOÀN TOÀN -> BỎ HẲN DEEP PULL, QUYẾT ĐỊNH STATE MỚI NGAY LẬP TỨC
                else {
                    if (localData.dist[0] < WARN_DIST) {
                        // Thay vì chuyển thẳng sang STRIKE (đâm ga tới đột ngột làm tăng dòng hộc và dễ trượt bánh),
                        // đưa về LOCK để tính toán góc steering bám đuổi mịn hơn hoặc setup đòn Counter-Rush.
                        enterState(STATE_ATK_LOCK);
                        DEBUG_PRINTLN(">>> EDGE CLEARED -> TARGET IN FRONT -> LOCK");
                    } else {
                        // Nếu trống trải, đẩy thẳng qua RECOVER. 
                        // Logic lùi chữ J (J-Turn) có sẵn trong RECOVER sẽ tự động làm nhiệm vụ đưa xe hướng về tâm sàn đấu một cách linh hoạt.
                        enterState(STATE_REC_RECOVER);
                        DEBUG_PRINTLN(">>> EDGE CLEARED -> DISENGAGE TO RECOVER");
                    }
                }

                // Ngắt khẩn cấp khi bị đâm văng trong lúc đang né vạch
                if (fsm_current_time - state_start_time > 80 && localData.impactDetected) {
                    enterState(STATE_ATK_STRIKE);
                    DEBUG_PRINTLN(">>> EDGE AVOID INTERRUPT: BỊ ĐÂM LÚC ĐANG NÉ -> TẤN CÔNG NƯỚC RÚT!");
                    break;
                }
                            
                // TIMEOUT FIX: Tránh lặp vô tận (Stuck prevention)
                if (fsm_current_time - state_start_time > 600) {
                    enterState(STATE_SEARCH_ENEMY);
                    DEBUG_PRINTLN(">>> EDGE AVOID STUCK! -> FORCE SEARCH");
                }
                break;
            }
            case STATE_DEF_LAST_STAND: // Khi xe bị lật nghiêng hoặc sắp rớt đài
            {
                if (localData.pitch < -PITCH_TH) driveBot(-PWM_MAX, -PWM_MAX); 
                else if (localData.pitch > PITCH_TH) driveBot(PWM_MAX, PWM_MAX);
                else if (localData.roll < -PITCH_TH) driveBot(-PWM_LOW, -PWM_MAX);
                else if (localData.roll > PITCH_TH) driveBot(-PWM_MAX, -PWM_LOW);
                else driveBot(-200, -200); 

                static uint32_t stable_time = 0;
                // Nếu lấy lại thăng bằng thành công -> Recover
                if (fabsf(localData.pitch) <= 5.0 && fabsf(localData.roll) <= 5.0 && !localData.fallOut) {
                    if (stable_time == 0) stable_time = fsm_current_time;
                    if (fsm_current_time - stable_time >= 100) {
                        enterState(STATE_REC_RECOVER);
                        stable_time = 0;
                        DEBUG_PRINTLN(">>> LAST STAND SURVIVED -> RECOVERING");
                    }
                } else {
                    stable_time = 0;
                }
                break;
            }

            case STATE_DEF_ANTI_LIFT:
            {
                uint32_t elapsed_time = fsm_current_time - state_start_time;

                // Mặc định: Hếch mũi -> lùi (-), Hếch đít -> tiến (+)
                int escape_pwm = localData.liftedFront ? -PWM_MAX : PWM_MAX; 
                
                // 1. TỰ CỨU MẠNG: CHỐNG RỚT ĐÀI OAN
                // Bị hếch mũi (đang lùi) mà đít giẫm vạch -> Tràn ga tiến tới!
                if (localData.liftedFront && (localData.line[2] <= TCRT_EDGE_TH || localData.line[3] <= TCRT_EDGE_TH)) {
                    escape_pwm = PWM_MAX; 
                    DEBUG_PRINTLN(">>> ANTI_LIFT: EDGE REAR DETECTED -> FORCE FORWARD!");
                }
                // Bị hếch đít (đang tiến) mà mũi giẫm vạch -> Tràn ga lùi kịch!
                else if (localData.liftedRear && (localData.line[0] <= TCRT_EDGE_TH || localData.line[1] <= TCRT_EDGE_TH)) {
                    escape_pwm = -PWM_MAX;
                    DEBUG_PRINTLN(">>> ANTI_LIFT: EDGE FRONT DETECTED -> FORCE REVERSE!");
                }

                // 2. KỊCH BẢN THOÁT GẦM CƯỜNG ĐỘ CAO
                if (elapsed_time < 150) {
                    driveBot(escape_pwm, escape_pwm); // Xả full ga giật ra khỏi gầm
                } 
                else if (elapsed_time < 500) { 
                    // Tăng tần số giật lên 50ms/pha. Tạo độ chênh lực gắt (255 vs 100) để văng đuôi
                    int phase = ((elapsed_time - 150) / 50) % 2; 
                    int weak_pwm = (escape_pwm > 0) ? 100 : -100; 
                    
                    if (phase == 0) driveBot(escape_pwm, weak_pwm); 
                    else driveBot(weak_pwm, escape_pwm);            
                } 
                else {
                    // DESPERATE PHASE: Vặn xoắn thân xe để phá vỡ cấu trúc cân bằng của nêm địch
                    driveBot(escape_pwm, -escape_pwm); 
                }

                // 3. ĐIỀU KIỆN THOÁT (Có bộ lọc nhiễu IMU)
                static uint32_t safe_ground_time = 0;
                if (!localData.liftedFront && !localData.liftedRear && fabsf(localData.pitch) <= 5.0 && fabsf(localData.roll) <= 5.0) {
                    if (safe_ground_time == 0) safe_ground_time = fsm_current_time;
                    
                    // Phải bám đất ổn định 150ms mới tính là thoát hẳn
                    if (fsm_current_time - safe_ground_time > 150) {
                        enterState(STATE_REC_RECOVER);
                        safe_ground_time = 0;
                        DEBUG_PRINTLN(">>> THOÁT KHỎI GẦM ĐỊCH -> RECOVERING!");
                    }
                } else {
                    safe_ground_time = 0; // Hủy đếm nếu xe lại bị giật nảy
                }
                
                // 4. TIMEOUT XUỐNG LAST_STAND (Kéo dài thêm để xe có thời gian giãy)
                if (elapsed_time > 800) {
                    enterState(STATE_DEF_LAST_STAND); 
                    DEBUG_PRINTLN(">>> ANTI_LIFT BẾ TẮC -> LAST STAND!");
                }
                
                break;
            }


            case STATE_DEF_SIDE_GUARD:
            {
                uint32_t elapsed_time = fsm_current_time - state_start_time;
                static int turn_dir = 1;
                static bool enemy_is_dangerously_close = false;

                // ENTRY ACTION: Xác định hướng địch và cự ly áp sát
                if (state_just_entered) {
                    turn_dir = (localData.enemy_angle < 0 || localData.dist[3] < localData.dist[4]) ? -1 : 1;
                    // Chốt cờ: Địch có đang ở vùng tử thần không?
                    enemy_is_dangerously_close = (localData.dist[3] < DIST_CLOSE || localData.dist[4] < DIST_CLOSE);
                }

                // SAFETY NET: Cắt đứt chuỗi nếu đạp vạch trắng
                if (localData.edgeDetect) {
                    enterState(STATE_DEF_EDGE_AVOID);
                    DEBUG_PRINTLN(">>> SIDE_GUARD INTERRUPT: EDGE DETECTED!");
                    break;
                }

                // PHẢN XẠ SỚM: Mắt trước (d0) bắt được địch -> Khóa ngay!
                if (elapsed_time > 50 && localData.dist[0] < WARN_DIST) {
                    enterState(STATE_ATK_LOCK);
                    DEBUG_PRINTLN(">>> SIDE_GUARD EARLY EXIT -> ENEMY FRONT -> ATK_LOCK");
                    break;
                }

                // PHASE TÁC CHIẾN ĐỘNG
                if (elapsed_time < 200) {
                    if (enemy_is_dangerously_close) {
                        // ĐỊCH SÁT SƯỜN: Buộc phải J-Turn lùi để xả lực ép và trượt nêm
                        if (turn_dir == -1) driveBot(-PWM_MAX, -PWM_MED);
                        else driveBot(-PWM_MED, -PWM_MAX);
                    } else {
                        // ĐỊCH CÒN XA: Không lùi hèn nhát, Pivot xoay vả mặt ngay lập tức
                        driveBot(PWM_MAX * turn_dir, -PWM_MAX * turn_dir);
                    }
                } 
                // PHASE 2: Quét vét máng (Dù xa hay gần cũng xoay nốt góc chết)
                else if (elapsed_time < 350) {
                    driveBot(PWM_MAX * turn_dir, -PWM_MAX * turn_dir);
                } 
                // PHASE 3: Đánh giá hậu quả
                else {
                    if (localData.dist[0] < WARN_DIST) {
                        enterState(STATE_ATK_LOCK); 
                        DEBUG_PRINTLN(">>> SIDE_GUARD SUCCESS -> ENEMY FRONT -> ATK_LOCK");
                    } 
                    else if (localData.dist[3] < DIST_CLOSE || localData.dist[4] < DIST_CLOSE) {
                        enterState(STATE_REC_RECOVER);
                        DEBUG_PRINTLN(">>> SIDE_GUARD STUCK (NASH EQUILIBRIUM) -> BREAK ORBIT!");
                    }
                    else if (localData.dist[1] < CONF_ENY || localData.dist[2] < CONF_ENY || 
                            localData.dist[3] < CONF_ENY || localData.dist[4] < CONF_ENY) {
                        enterState(STATE_ATK_LOCK);
                        DEBUG_PRINTLN(">>> SIDE_GUARD -> ENEMY MOVED AWAY");
                    }
                    else {
                        enterState(STATE_SEARCH_ENEMY); 
                        DEBUG_PRINTLN(">>> SIDE_GUARD DONE -> SEARCHING");
                    }
                }
                break;
            }

            case STATE_DEF_REAR_GUARD:
            {
                uint32_t elapsed_time = fsm_current_time - state_start_time;
                static int turn_dir = 1;
                static float start_yaw = 0.0;

                // ENTRY ACTION: Xác định hướng xoay để đối mặt ngay với đối thủ
                if (state_just_entered) {
                    // Nếu enemy_angle < 0 (Nửa trái), xoay gắt sang Trái (turn_dir = -1) để đưa mặt lại nhanh nhất
                    turn_dir = (localData.enemy_angle < 0) ? -1 : 1;
                    start_yaw = localData.yaw;
                }

                // 1. PHẢN XẠ SỚM: Nếu trong lúc xoay mà mắt trước (d0) đã bắt được địch
                if (localData.dist[0] < WARN_DIST) {
                    enterState(STATE_ATK_LOCK);
                    DEBUG_PRINTLN(">>> REAR_GUARD: ĐÃ ĐỐI MẶT ĐỊCH -> ATK_LOCK!");
                    break;
                }

                // 2. VÒNG LẶP XOAY GẮT (PIVOT TẠI CHỖ)
                // Theo dõi IMU để xoay cho đến khi góc lệch đạt 180 độ
                if (fabsf(localData.yaw - start_yaw) < 180.0) {
                    // Pivot tại chỗ: 1 bánh tiến full ga, 1 bánh lùi full ga
                    driveBot(PWM_MAX * turn_dir, -PWM_MAX * turn_dir);
                    
                    // Timeout dự phòng nếu xe bị kẹt lực ma sát với đối thủ không xoay được
                    if (elapsed_time > 800) {
                        enterState(STATE_REC_RECOVER);
                        DEBUG_PRINTLN(">>> REAR_GUARD TIMEOUT (STUCK) -> RECOVERING");
                    }
                } 
                else {
                    // Đã xoay đủ 180 độ nhưng địch đã lách đi chỗ khác (hụt d0)
                    if (localData.dist[1] < CONF_ENY || localData.dist[2] < CONF_ENY || 
                        localData.dist[3] < CONF_ENY || localData.dist[4] < CONF_ENY) {
                        enterState(STATE_ATK_LOCK);
                        DEBUG_PRINTLN(">>> REAR_GUARD (180deg) -> THẤY ĐỊCH BÊN HÔNG -> FLANK SIDE");
                    } else {
                        enterState(STATE_SEARCH_ENEMY);
                        DEBUG_PRINTLN(">>> REAR_GUARD (180deg) -> MẤT DẤU -> SEARCHING");
                    }
                }
                break;
            }

            case STATE_SEARCH_ENEMY:
            {
                uint32_t elapsed_time = fsm_current_time - state_start_time;

                // Spiral/Widen Search
                static int search_dir = 1; 

                if (state_just_entered) {
                     search_dir = (localData.enemy_angle < 0) ? -1 : 1; // Ưu tiên quay về phía vừa mất dấu
                }

                // PHẢN XẠ SĂN ĐUÔI: ĐỊCH CHẮC CHẮN Ở SAU LƯNG KHI MÙ HOÀN TOÀN
                if (localData.isTargetLost && previousState != STATE_DEF_REAR_GUARD) {
                    
                    // KỊCH BẢN 1: Vừa rớt khỏi chuỗi TẤN CÔNG mà mất dấu ngay lập tức
                    // Chứng tỏ địch vừa tung đòn lách né (Side-step) hoặc Glance-blow ra sau lưng bạn.
                    // Phản xạ: Quay ngoắt 180 độ cứu vãn tình thế NGAY TRONG 0MS.
                    if (previousState == STATE_ATK_LOCK || previousState == STATE_ATK_STRIKE) {
                        enterState(STATE_DEF_REAR_GUARD);
                        DEBUG_PRINTLN(">>> SEARCH: ĐỊCH LÁCH RA SAU KHI CỦA CÔNG -> XOAY 180 ĐỘ ĐÓN LÕNG!");
                        break;
                    }
                    
                    // KỊCH BẢN 2: Đang quét tìm kiếm bình thường mà hoàn toàn mù suốt 150ms
                    // Khoảng thời gian 150ms (khoảng 4-5 chu kỳ lấy mẫu ToF) đủ để khẳng định 
                    // đây không phải nhiễu cảm biến tạm thời. Địch chắc chắn bám đuôi!
                    else if (elapsed_time > 150) {
                        enterState(STATE_DEF_REAR_GUARD);
                        DEBUG_PRINTLN(">>> SEARCH: QUÉT 150MS VẪN MÙ -> ĐỊCH CHẮC CHẮN Ở SAU -> REAR GUARD!");
                        break;
                    }
                }

                // Nếu không thỏa mãn các điều kiện mù ở sau, tiếp tục quét xoay ốc như cũ
                if (elapsed_time < 600) {
                    driveBot(150 * search_dir, -150 * search_dir); // Xoay tại chỗ quét nhanh
                } 
                else if (elapsed_time < 1500) {
                    driveBot(200 * search_dir, 50 * search_dir);  // Quét vòng cung rộng
                } 
                else {
                    search_dir = -search_dir; 
                    state_start_time = fsm_current_time; // Bắt đầu chu kỳ tìm kiếm mới
                    Serial.print(">>> SEARCH TIMEOUT -> REVERSE DIRECTION (Dir: ");
                    Serial.print(search_dir);
                    DEBUG_PRINTLN(")");
                }

                // Tìm thấy mục tiêu (Giữ nguyên các phản xạ phản công phía trước mặt)
                if (localData.dist[0] < CONF_ENY) {
                    if (localData.dist[0] <= WARN_DIST || localData.closingFast) {
                        enterState(STATE_ATK_STRIKE);
                        DEBUG_PRINTLN(">>> SEARCH: ĐỊCH LAO NHANH / Ở GẦN -> RUSH BỎ QUA LOCK!");
                    } else {
                        enterState(STATE_ATK_LOCK);
                        DEBUG_PRINTLN(">>> SEARCH: FOUND IN FRONT (FAR) -> LOCK");
                    }
                }
                else if (localData.dist[3] < DIST_CLOSE || localData.dist[4] < DIST_CLOSE) {
                    enterState(STATE_DEF_SIDE_GUARD);
                    DEBUG_PRINTLN(">>> SEARCH: DANGER AT DIRECT SIDE (d3/d4) -> SIDE GUARD EVASION!");
                }
                else if (localData.dist[1] < CONF_ENY || localData.dist[2] < CONF_ENY || 
                         localData.dist[3] < CONF_ENY || localData.dist[4] < CONF_ENY) {
                    enterState(STATE_ATK_LOCK);
                    DEBUG_PRINTLN(">>> SEARCH: FOUND AT DIAGONAL/FAR SIDE -> FLANK SIDE");
                }
                break;
            }

            case STATE_REC_RECOVER:
            {
                static int turn_dir = 1; 
                uint32_t elapsed_time = fsm_current_time - state_start_time;

                // ENTRY ACTION: Khởi tạo hướng lượn lốc xoáy
                if (state_just_entered) {
                    // Ưu tiên quay về phía vừa mất dấu, nếu mù hoàn toàn thì tung đồng xu
                    turn_dir = (!localData.isTargetLost && localData.enemy_angle < 0) ? -1 : 1;
                    if (localData.isTargetLost) turn_dir = (esp_random() & 1) ? 1 : -1;
                }

                // 1. EARLY EXIT 1: BỊ ĐÂM - LẤY CÔNG LÀM THỦ
                // Đang recover mà bị húc -> Không nhân nhượng, tràn ga húc lại ngay
                if (elapsed_time > 50 && localData.impactDetected) {
                    enterState(STATE_ATK_STRIKE);
                    DEBUG_PRINTLN(">>> OFFENSIVE RECOVER: BỊ ĐÂM -> BẬT LẠI STRIKE!");
                    break;
                }

                // 2. EARLY EXIT 2: ĐẠP VẠCH TRƯỚC
                // Vì bây giờ ta liên tục lao lên, rủi ro giẫm vạch trước cao hơn vạch đuôi
                if (localData.edgeDetect) {
                    enterState(STATE_DEF_EDGE_AVOID);
                    DEBUG_PRINTLN(">>> OFFENSIVE RECOVER: THẤY VẠCH -> AVOID!");
                    break;
                }

                // 3. EARLY EXIT 3: PHẢN XẠ CHỚP NHOÁNG (Chỉ quét sau 50ms chống nhiễu)
                if (elapsed_time > 50) {
                    // Địch lọt vào chính diện -> Khoá mục tiêu
                    if (localData.dist[0] < WARN_DIST) {
                        enterState(STATE_ATK_LOCK);
                        DEBUG_PRINTLN(">>> OFFENSIVE RECOVER: THẤY ĐỊCH CHÍNH DIỆN -> LOCK!");
                        break;
                    }
                    // Địch lọt vào 2 bên hông -> Trượt sườn ngay
                    else if (localData.dist[1] < WARN_DIST || localData.dist[2] < WARN_DIST || 
                            localData.dist[3] < WARN_DIST || localData.dist[4] < WARN_DIST) {
                        enterState(STATE_ATK_LOCK);
                        DEBUG_PRINTLN(">>> OFFENSIVE RECOVER: THẤY ĐỊCH BÊN HÔNG");
                        break;
                    }
                }

                // 4. EARLY EXIT 4: MÙ LÀ ĐỊCH Ở SAU LƯNG
                // Nếu xoay gắt 150ms rồi mà ToF vẫn không thấy ai -> Khả năng cao địch bám đuôi
                if (elapsed_time > 150 && localData.isTargetLost) {
                    enterState(STATE_DEF_REAR_GUARD);
                    DEBUG_PRINTLN(">>> OFFENSIVE RECOVER: BLIND -> ĐỊCH Ở SAU LƯNG -> REAR GUARD!");
                    break;
                }

                // 5. ĐIỀU KHIỂN MOTOR (Kinematics bám đuổi)
                // Pha 1 (0 -> 150ms): Pivot tại chỗ MAX tốc độ để xé góc mù
                if (elapsed_time < 150) {
                    driveBot(PWM_MAX * turn_dir, -PWM_MAX * turn_dir);
                }
                // Pha 2 (150ms -> MAX_RECOVER_TIME): Forward Arc - Tiến tới lượn vòng chiếm sân
                else if (elapsed_time < MAX_RECOVER_TIME) {
                    // Vi sai: 1 bánh đẩy full ga, 1 bánh đẩy ga trung bình (150)
                    if (turn_dir == 1) {
                        driveBot(PWM_MAX, PWM_MED); 
                    } else {
                        driveBot(PWM_MED, PWM_MAX);
                    }
                }

                // --- 6. TIMEOUT DỰ PHÒNG ---
                if (elapsed_time >= MAX_RECOVER_TIME) {
                    enterState(STATE_SEARCH_ENEMY);
                    DEBUG_PRINTLN(">>> OFFENSIVE RECOVER TIMEOUT -> SEARCH");
                }
                
                break;
            }

            // NHÓM 3: TẤN CÔNG (ATTACK)
            case STATE_ATK_LOCK:
            {
                // Bám mục tiêu
                static uint8_t lock_retries = 0;
                uint32_t elapsed_time = fsm_current_time - state_start_time;

                if (state_just_entered) stalemate_cycles = 0;

                // PHẢN XẠ TẠT MÚ KHI ĐỊCH LÁCH GẮT / MẤT DẤU CHỚP NHOÁNG (< 200ms)
                if (localData.isTargetLost) {
                    
                    // Nếu thời gian mất dấu nhỏ hơn 200ms -> Lập tức tung đòn giả để quét sườn đánh chặn
                    if (lost_duration < 200) {
                        enterState(STATE_ATK_FEINT);
                        failed_strike_count = 0; // Xóa đếm để chuẩn bị chu kỳ combat mới
                        lock_retries = 0;
                        DEBUG_PRINTLN(">>> TARGET FLASH-LOST (<200ms) -> REFLEX FEINT!");
                        break;
                    }

                    // Nếu mất dấu lâu hơn 200ms -> Chuyển sang cơ chế dừng xe ngắm lại / tìm kiếm nguyên bản
                    lock_retries++;
                    driveBot(0, 0); 

                    if (lock_retries >= MAX_LOCK_RETRIES) {
                        enterState(STATE_SEARCH_ENEMY);
                        lock_retries = 0;
                        DEBUG_PRINTLN(">>> LOCK FAILED: LOST TARGET -> SEARCH");
                    } else {
                        state_start_time = fsm_current_time; // Reset timer để thử ngắm lại
                    }
                    break;
                }

                // THOÁT KHẨN CẤP KHI GÓC QUÁ GẮT (> 60 ĐỘ)
                // Nếu địch đã lọt ra khỏi hình nón phía trước, việc xoay tại chỗ là tự sát.
                if (fabsf(localData.enemy_angle) > 60.0) {
                    // Nếu địch áp sát sườn -> Tung bài lùi chữ J thủ sườn
                    if (localData.dist[3] < WARN_DIST || localData.dist[4] < WARN_DIST) {
                        enterState(STATE_DEF_SIDE_GUARD);
                        DEBUG_PRINTLN(">>> LOCK BAILOUT: ĐỊCH SÁT HÔNG (>60 độ) -> J-TURN THỦ SƯỜN!");
                    } 
                    // Nếu địch ở xa -> Kích hoạt cơ động bọc lót tạt sườn
                    else {
                        enterState(STATE_SEARCH_ENEMY);
                        DEBUG_PRINTLN(">>> LOCK BAILOUT: ĐỊCH NGOÀI TẦM NGẮM (>60 độ) -> ĐÁNH CHẶN SƯỜN!");
                    }
                    break;
                }

                // ĐIỀU HƯỚNG MỤC TIÊU VÀO CHÍNH DIỆN
                float err_angle = localData.enemy_angle; 
                int forward_pwm = 0;
                int turn_pwm = 0;

                // Triệt tiêu độ giật khi góc dao động quanh ngưỡng TIGHT
                if (fabsf(err_angle) <= ANGLE_TIGHT) {
                    driveBot(PWM_HIGH, PWM_HIGH); // Thẳng tắp -> Phóng thẳng
                } else {
                    // Chỉ tính turn_pwm khi ngoài vùng ANGLE_TIGHT
                    turn_pwm = constrain(fabsf(err_angle) * KP_STEERING, PWM_TURN_MIN, PWM_HIGH); 
                    
                    if (fabsf(err_angle) < ANGLE_FLANK) { 
                        forward_pwm = PWM_MED; // Vẫn đang nhìn thấy khá rõ -> Vừa tiến vừa bẻ
                    } else {
                        forward_pwm = 0; // Lệch góc quá gắt -> Xoay tại chỗ để bắt hình nhanh
                    }

                    if (err_angle > 0) {
                        driveBot(forward_pwm + turn_pwm, forward_pwm - turn_pwm);
                    } else {
                        driveBot(forward_pwm - turn_pwm, forward_pwm + turn_pwm);
                    }
                }

                // Phát hiện bế tắc: địch sát mặt nhưng không tiến triển
                if (localData.dist[0] < 150 && fabsf(localData.v_e) < 50 && elapsed_time > 500) {
                    enterState(STATE_ATK_STRIKE);
                    DEBUG_PRINTLN(">>> LOCK STALEMATE -> FORCE STRIKE!");
                    break;
                }

                // TÍNH TOÁN KHOẢNG CÁCH TẤN CÔNG ĐỘNG
                uint16_t dynamic_strike_dist = STRIKE_DIST; 

                if (localData.closingFast) {
                    // Nếu địch cũng đang rồ ga lao tới -> Kích hoạt đòn húc từ XA (vd: 650mm) 
                    // để lấy đà và đón lõng
                    dynamic_strike_dist = 650; 
                } else if (localData.v_e < 50.0) {
                    // Nếu địch lỳ đòn đứng im hoặc đang lùi -> Phải rúc vào GẦN (vd: 350mm) 
                    // mới được bung lực để tránh bị lừa đòn
                    dynamic_strike_dist = 350; 
                }

                // KIỂM TRA ĐIỀU KIỆN RA ĐÒN
                bool is_ready_to_strike = false;
                if (localData.dist[0] < dynamic_strike_dist && fabsf(err_angle) <= ANGLE_WIDE) {
                    is_ready_to_strike = true;
                } 
                else if (localData.dist[0] < DIST_CLOSE && fabsf(err_angle) <= ANGLE_WIDE) {
                    // Tuyệt đối không được STRIKE. Ép nó tạt hông hoặc vờn trước mặt để bắt lại góc thẳng!
                    enterState(STATE_ATK_LOCK); 
                    DEBUG_PRINTLN(">>> LOCK WARNING: QUÁ GẦN NHƯNG LỆCH GÓC NẶNG");
                    break;
                }

                if (is_ready_to_strike) {
                    if (!localData.sideDanger) {
                        
                        // Cây quyết định chiến thuật (Decision Tree)
                        if (localData.closingFast) {
                            enterState(STATE_ATK_DELAY_RUSH); // Địch đang lao tới nhanh -> Bẩy Judo
                            lock_retries = 0;
                            DEBUG_PRINTLN(">>> LOCK SUCCESS -> ENEMY RUSHING -> DELAY RUSH!");
                        } 
                        else {
                            // PHÁ THẾ BÁM SÀN HOẶC TACTICAL FEINT
                            if (failed_strike_count >= 2 || lock_retries >= 2) {
                                if (fabsf(localData.v_e) < 50.0) {
                                    // Địch lỳ đòn, vận tốc cực nhỏ -> Đưa thẳng về RECOVER
                                    enterState(STATE_REC_RECOVER);
                                    DEBUG_PRINTLN(">>> LOW V_E DETECTED -> RECOVER!");
                                } else {
                                    // Địch vẫn di chuyển nhanh -> Lừa đòn
                                    enterState(STATE_ATK_FEINT);
                                    DEBUG_PRINTLN(">>> 2 STRIKES FAILED -> TACTICAL FEINT!");
                                }
                                failed_strike_count = 0; 
                                lock_retries = 0;
                            } else {
                                enterState(STATE_ATK_STRIKE); // Đâm thẳng mặt mặc định nếu chưa đủ điều kiện
                                lock_retries = 0; 
                                DEBUG_PRINTLN(">>> LOCK SUCCESS -> DEFAULT STRIKE!");
                            }
                        }
                    }
                }

                // Bổ sung: Thoát khẩn cấp sang tạt sườn nếu địch hoàn toàn ở bên hông (90 độ) mà trước mặt trống
                if (localData.dist[0] >= CONF_ENY && (localData.dist[3] < CONF_ENY || localData.dist[4] < CONF_ENY)) {
                    enterState(STATE_ATK_LOCK);
                    DEBUG_PRINTLN(">>> LOCK: ENEMY AT 90 DEG AND FRONT EMPTY");
                    break;
                }

                // Ưu tiên cao: tránh rơi khỏi sàn
                if (localData.edgeDetect && localData.dist[0] < WARN_DIST) {
                    enterState(STATE_DEF_EDGE_AVOID);
                    DEBUG_PRINTLN(">>> LOCK: EDGE RISK -> AVOID!");
                    break;
                }
                
                // KIỂM SOÁT TIMEOUT CỰC ĐOAN (Bế tắc vật lý - Giữ nguyên)
                if (elapsed_time > ATK_LOCK_TIME) {
                    if (localData.dist[0] < CONF_ENY) {
                        enterState(STATE_ATK_STRIKE);
                        DEBUG_PRINTLN(">>> LOCK TIMEOUT -> DESPERATE STRIKE");
                    } else {
                        enterState(STATE_SEARCH_ENEMY);
                        DEBUG_PRINTLN(">>> LOCK TIMEOUT -> MẤT DẤU -> SEARCH");
                    }
                    lock_retries = 0;
                }
                
                break;
            }

            case STATE_ATK_STRIKE:
            {
                // Chuỗi ra đòn
                uint32_t elapsed_time = fsm_current_time - state_start_time;

                // Tới công chuyện luôn, full ga đâm thẳng không lươn lẹo
                driveBot(PWM_MAX, PWM_MAX); 

                // KỊCH BẢN TRƯỢT GÓC (GLANCING BLOW / SLIPPED)
                // Đang đẩy nhưng góc lệch to, tức là xe đang bị văng sườn[cite: 3]
                if (elapsed_time > 500) { // Tăng lên 500ms để bao trọn thời gian tiếp cận và va chạm[cite: 3]
                    if (fabsf(localData.enemy_angle) > 25.0) {
                        enterState(STATE_ATK_LOCK);
                        failed_strike_count++; 
                        DEBUG_PRINTLN(">>> STRIKE SLIPPED -> BÁM LẠI GÓC!");
                        break;
                    }
                }

                // Nếu hất tung được địch -> Cú STRIKE cực kì thành công -> Reset bộ đếm[cite: 3]
                if (localData.liftDetected) {
                    failed_strike_count = 0; 
                    enterState(STATE_ATK_LIFT);
                    DEBUG_PRINTLN(">>> STRIKE -> ENEMY LIFTED -> ATK_LIFT (FULL POWER)!"); 
                    break;
                }

                // STALEMATE
                // Nếu hai xe đang húc nhau giằng co quá lâu (vượt TIMEOUT_MAX),
                // Kích hoạt tự hãm của Worm Gear bằng cách phanh cứng 0 PWM.[cite: 3]
                if (elapsed_time > TIMEOUT_MAX) {
                    // Giằng co hết thời gian mà v_e < 50 -> Địch bám sàn quá tốt -> Fail[cite: 3]
                    if (localData.v_e < 50.0) failed_strike_count++; 
                    
                    enterState(STATE_ATK_STALEMATE_BRAKE);
                    DEBUG_PRINTLN(">>> STALEMATE TIMEOUT -> WORM GEAR BRAKE (STAND YOUR GROUND)!");
                }

                break;
            }
            
            case STATE_ATK_STALEMATE_BRAKE: // Khoá cứng động cơ Worm Gear, lợi dụng cơ khí trục vít tự hãm
            {
                uint32_t elapsed_time = fsm_current_time - state_start_time;

                // Ép PWM = 0 để khóa cứng bánh răng (Only for Worm Gear)
                driveBot(0, 0);

                // Đứng im trong 250ms 
                if (elapsed_time >= 250) {
                    if (localData.dist[0] <= WARN_DIST && fabsf(localData.enemy_angle) <= 15.0) {
                        
                        // Địch vẫn lù lù trước mặt -> Tăng biến đếm bế tắc
                        stalemate_cycles++; 
                        
                        // --- LOGIC MỚI: TỔNG TẤN CÔNG PHÁ BẾ TẮC ---
                        if (stalemate_cycles >= 2) {
                            stalemate_cycles = 0; 
                            
                            // Địch lỳ quá?
                            enterState(STATE_ATK_LOCK); 
                            DEBUG_PRINTLN(">>> STALEMATE BROKEN -> ENEMY TOO STRONG");
                        } else {
                            enterState(STATE_ATK_STRIKE);
                            DEBUG_PRINTLN(">>> BRAKE DONE -> RE-STRIKE!");
                        }
                    } else {
                        // Địch bị trượt hoặc lùi lại -> Nhìn lại góc cho chuẩn
                        stalemate_cycles = 0; // Reset bộ đếm
                        enterState(STATE_ATK_LOCK);
                        DEBUG_PRINTLN(">>> BRAKE DONE -> ENEMY SLIPPED -> RE-LOCK");
                    }
                }
                break;
            }

            case STATE_ATK_LIFT:
            {
                uint32_t elapsed_time = fsm_current_time - state_start_time; 
                driveBot(PWM_MAX, PWM_MAX);
                
                // 1. Chống tự sát: Thấy vạch trắng phải dừng nhấc và lo giữ mạng
                if (localData.edgeDetect) {
                    enterState(STATE_DEF_EDGE_AVOID);
                    DEBUG_PRINTLN(">>> ATK_LIFT: EDGE DETECTED -> ABORT AND AVOID!");
                    break;
                }

                // 2. Chống kẹt / Timeout: Nếu nhấc đối thủ quá 2000ms hoặc mục tiêu bị rớt thì buông ra
                if (!localData.liftDetected || elapsed_time > 2000) { 
                    if (localData.dist[0] < WARN_DIST) {
                        enterState(STATE_ATK_STRIKE);
                    } else {
                        enterState(STATE_SEARCH_ENEMY);
                    }
                    DEBUG_PRINTLN(">>> ATK_LIFT: TIMEOUT OR DROPPED -> NEXT STATE");
                }
                break;
            }

            case STATE_ATK_FEINT:
            {
                uint32_t elapsed_time = fsm_current_time - state_start_time;

                if (elapsed_time < 400) {
                    driveBot(180, 180);
                } else {
                    if (localData.enemy_angle < 0) driveBot(-PWM_LOW, PWM_MAX); 
                    else driveBot(PWM_MAX, -100);
                }

                if (elapsed_time >= 450) { 
                    enterState(STATE_ATK_LOCK); 
                    DEBUG_PRINTLN(">>> FEINT COMPLETE");
                }
                if (localData.impactDetected) {
                    enterState(STATE_ATK_STRIKE);
                    DEBUG_PRINTLN(">>> FEINT IMPACT EARLY -> STRIKE");
                }
                if (localData.dist[0] > CONF_ENY && elapsed_time > 100) {
                    enterState(STATE_SEARCH_ENEMY);
                    DEBUG_PRINTLN(">>> FEINT MISSED -> SEARCH");
                }
                break;
            }

            case STATE_ATK_DELAY_RUSH:
            {
                uint32_t elapsed_time = fsm_current_time - state_start_time;

                // Bẩy Judo (Counter-Rush)
                // KHÔNG đứng im. Bơm ga nhẹ (PWM = 80) để động cơ có mô-men xoắn,
                // ép chặt lưỡi ủi Teflon sát rạt xuống mặt sàn, biến xe thành một cái nêm.
                driveBot(80, 80); 

                // Ngay khi địch lọt vào 150, dậm kịch ga để bẩy
                if (localData.dist[0] <= 150) {
                    enterState(STATE_ATK_STRIKE); 
                    DEBUG_PRINTLN(">>> COUNTER RUSH: ĐỊCH VÀO TẦM (<150mm) -> BUNG MAX GA (UPPERCUT)!");
                }
                // Nếu địch lươn lẹo bẻ lái sang hướng khác (lệch góc > 20 độ) -> Hủy rình, ngắm lại
                else if (fabsf(localData.enemy_angle) > 20.0) {
                    enterState(STATE_ATK_LOCK); 
                    DEBUG_PRINTLN(">>> COUNTER CANCELLED: ĐỊCH LÁCH GÓC -> RE-LOCK");
                }
                // Nếu địch đột ngột nhát gan phanh lại hoặc đi chậm (v_e <= 100) -> Bỏ rình, chủ động lao lên atk
                else if (localData.v_e <= 100.0) { 
                    enterState(STATE_ATK_STRIKE); 
                    DEBUG_PRINTLN(">>> COUNTER CANCELLED: ĐỊCH CHẬM LẠI -> CHỦ ĐỘNG STRIKE");
                }
                // Bế tắc thời gian (Đề phòng 2 xe gằm ghè nhau ngoài tầm 150mm quá lâu)
                else if (elapsed_time > 600) {
                    enterState(STATE_ATK_STRIKE);
                    DEBUG_PRINTLN(">>> COUNTER TIMEOUT -> ÉP XUNG ĐÁNH BỪA");
                }
                
                break;
            }
        }
        static RobotState last_processed_state = STATE_IDLE;
        if (last_processed_state == currentState) {
            state_just_entered = false; 
        }
        last_processed_state = currentState;
        

        // EVENT-DRIVEN DELAY
        // FSM sẽ "ngủ" tối đa 5ms. 
        // - NẾU trong 5ms đó không có gì xảy ra -> Nó tự thức dậy chạy loop bình thường (Polling backup).
        // - NẾU TaskSensor vừa gọi xTaskNotifyGive() -> Nó bật dậy NGAY LẬP TỨC (tốc độ ánh sáng).
        // Lệnh này vừa thay thế vTaskDelay(), vừa dọn dẹp Notification flag.
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5));
    }
}

void enterState(RobotState newState) {
    // --- LỌC NHIỄU (DEBOUNCE) STATE ---
    // Tránh tình trạng bot nhảy liên tục giữa các trạng thái trong vài ms
    if (newState != STATE_DEF_LAST_STAND && 
        newState != STATE_DEF_EDGE_AVOID && 
        newState != STATE_DEF_ANTI_LIFT &&
        newState != STATE_REC_RECOVER) {
        
        if (millis() - state_start_time < MIN_STT_TIME) {
            return; // Bỏ qua lệnh chuyển trạng thái này
        }
    }

    RobotState oldState = currentState; // Giữ lại state cũ để so sánh
    previousState = oldState;

    currentState = newState;
    state_start_time = millis(); 
    state_just_entered = true;

    // --- LOGIC OLED ---
    if (oldState == STATE_INIT_DELAY) {
        go_lock = true;             // Bật cờ ưu tiên GO!
        go_start_time = millis();   // Chốt mốc thời gian 500ms
    } 
    else if (newState == STATE_DEF_LAST_STAND) {
        go_lock = false;            // Hủy GO! nếu xe bị lật ngay lập tức
    }
    
    // Luôn báo cho loop() biết FSM vừa chuyển state để cập nhật màn hình
    needsDisplayUpdate = true; 
}

float getModeAngle(float* history, int size) {
    int maxCount = 0;
    float modeVal = 999;
    
    // Tạo mảng tạm để lưu giá trị Bucket (đã làm tròn)
    int buckets[50];
    for (int i = 0; i < size; i++) {
        // Ví dụ: góc 12.3 độ và 14.1 độ đều được ném chung vào bucket số 2 (2 * 5 = 10 độ)
        buckets[i] = (int)(history[i] / ANGLE_BIN_RES); 
    }

    // Tìm Mode dựa trên số nguyên (an toàn tuyệt đối)
    for (int i = 0; i < size; i++) {
        if (history[i] == 999.0) continue;
        int count = 0;
        for (int j = 0; j < size; j++) {
            if (buckets[j] == buckets[i]) count++;
        }
        if (count > maxCount) {
            maxCount = count;
            // Mode Value trả về giá trị float gốc của phần tử đại diện cho Bucket đó
            modeVal = history[i]; 
        }
    }
    return modeVal;
}

const char* getStateName(RobotState state) {
    switch(state) {
        case STATE_IDLE: return "IDLE";
        case STATE_INIT_DELAY: return "INIT_DELAY";
        case STATE_ATK_STRIKE: return "STRIKE";
        case STATE_ATK_LIFT: return "ATK_LIFT";
        case STATE_ATK_FEINT: return "FEINT";
        case STATE_ATK_DELAY_RUSH: return "DELAY_RUSH";
        case STATE_ATK_LOCK: return "LOCK";
        case STATE_ATK_STALEMATE_BRAKE: return "BRAKE";
        case STATE_DEF_SIDE_GUARD: return "SIDE_GUARD";
        case STATE_DEF_REAR_GUARD: return "REAR_GUARD";
        case STATE_DEF_EDGE_AVOID: return "EDGE_AVOID";
        case STATE_DEF_ANTI_LIFT: return "ANTI_LIFT";
        case STATE_DEF_LAST_STAND: return "LAST_STAND";
        case STATE_REC_RECOVER: return "RECOVER";
        case STATE_SEARCH_ENEMY: return "SEARCH";
        case STATE_CALIBRATION: return "CALIB_MODE";
        default: return "UNKNOWN";
    }
}



