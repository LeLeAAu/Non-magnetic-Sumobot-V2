#include "Config.h"
#include "MotorControl.h"
#include "DisplayFace.h"
#include "SensorTask.h"
#include "FSMTask.h"

float getModeAngle(float* history, int size);

void TaskFSMCode(void * pvParameters) {
    // Các biến tĩnh dùng riêng cho FSM
    
    // Biến cho việc build Histogram trong lúc chờ
    const int HIST_SIZE = 50;
    static float angle_histogram[HIST_SIZE];

    for (int i = 0; i < HIST_SIZE; i++) {
        angle_histogram[i] = 999.0;
    }
    static int hist_idx = 0;
    static uint8_t stalemate_cycles = 0; // Đếm số lần bế tắc

    for(;;) {
        uint32_t fsm_current_time = millis();

        // HỤP ẢNH DỮ LIỆU CẢM BIẾN 
        SystemData localData; // Bản sao cục bộ chỉ sống trong vòng lặp này
        
        xSemaphoreTake(dataMutex, portMAX_DELAY); // Khóa Mutex an toàn
        localData = sysData;          
        xSemaphoreGive(dataMutex); // Mở khóa
        // TỪ ĐÂY TRỞ XUỐNG, CHỈ SỬ DỤNG localData. TUYỆT ĐỐI KHÔNG GỌI sysData NỮA!

        // [GLOBAL SAFETY LAYER]
        bool is_self_jerk_blind_time = (fsm_current_time - state_start_time < 250);

        if (currentState != STATE_IDLE && currentState != STATE_INIT_DELAY) {

            // PRIORITY 1: LẬT XE / RỚT ĐÀI
            if (localData.fallOut) {
                if (currentState != STATE_DEF_LAST_STAND) {
                    enterState(STATE_DEF_LAST_STAND);
                    Serial.println(">>> GLOBAL SAFETY: FALL OUT -> LAST STAND!");
                }
            }
            // PRIORITY 2: BỊ NHẤC BỔNG / XÚC GẦM
            else if (localData.liftedFront || localData.liftedRear || localData.beingLifted) {
                if (currentState != STATE_DEF_ANTI_LIFT) {
                    enterState(STATE_DEF_ANTI_LIFT);
                    Serial.println(">>> GLOBAL SAFETY: BEING LIFTED -> ANTI_LIFT!");
                }
            }
            // PRIORITY 3: MÉP SÂN
            else if (localData.edgeDetect) {
                if (currentState != STATE_DEF_EDGE_AVOID && currentState != STATE_DEF_LAST_STAND) {
                    enterState(STATE_DEF_EDGE_AVOID);
                    Serial.println(">>> GLOBAL SAFETY: EDGE DETECTED -> AVOID!");
                }
            }
            // PRIORITY 4: BỊ ĐÂM CHÍ MẠNG (Phản công)
            else if (localData.impactDetected && !is_self_jerk_blind_time && 
                     currentState != STATE_ATK_STRIKE && 
                     currentState != STATE_ATK_LIFT &&
                     currentState != STATE_DEF_ANTI_PUSH &&
                     currentState != STATE_DEF_REAR_GUARD &&
                     currentState != STATE_DEF_SIDE_GUARD) {
                
                bool blind_hit = (localData.dist[0] > WARN_DIST && localData.dist[1] > WARN_DIST && 
                                  localData.dist[2] > WARN_DIST && localData.dist[3] > WARN_DIST && localData.dist[4] > WARN_DIST);
                bool side_hit = (localData.dist[3] < WARN_DIST || localData.dist[4] < WARN_DIST);

                if (blind_hit) {
                    enterState(STATE_DEF_REAR_GUARD);
                    Serial.println(">>> GLOBAL SAFETY: BLIND IMPACT -> REAR GUARD!");
                } else if (side_hit && currentState == STATE_SEARCH_ENEMY) {
                    enterState(STATE_DEF_SIDE_GUARD);
                    Serial.println(">>> GLOBAL SAFETY: SIDE IMPACT -> SIDE GUARD!");
                } else {
                    enterState(STATE_DEF_ANTI_PUSH);
                    Serial.println(">>> GLOBAL SAFETY: FRONT IMPACT -> ANTI_PUSH!");
                }
            }
        }

        // [NORMAL FSM SWITCH-CASE]
        switch (currentState) {
            
            // NHÓM 1: KHỞI TẠO (INIT)
            case STATE_IDLE:
            {
                driveBot(0, 0);
                
                if (!localData.isTargetLost) {
                    angle_histogram[hist_idx] = localData.enemy_angle;
                    hist_idx = (hist_idx + 1) % HIST_SIZE;
                }

                if (digitalRead(PIN_TTP223) == HIGH) {
                    enterState(STATE_INIT_DELAY);
                    Serial.println(">>> START: INIT_DELAY (3 seconds)");
                } 
                break;
            }

            case STATE_INIT_DELAY: // Gộp chung logic đọc Histogram cho gọn
            {
                driveBot(0, 0);
                
                if (!localData.isTargetLost) {
                    angle_histogram[hist_idx] = localData.enemy_angle;
                    hist_idx = (hist_idx + 1) % HIST_SIZE;
                } else {
                    // CẢ 5 MẮT ĐỀU THẤY TRỐNG TRƠN (> 2000mm) -> ĐỊCH Ở SAU LƯNG!
                    if (localData.dist[0] > DIST_BLIND && localData.dist[1] > DIST_BLIND && 
                        localData.dist[2] > DIST_BLIND && localData.dist[3] > DIST_BLIND && 
                        localData.dist[4] > DIST_BLIND) {
                        
                        angle_histogram[hist_idx] = 180.0; // Ném góc 180 độ vào Data
                        hist_idx = (hist_idx + 1) % HIST_SIZE;
                    }
                }

                if (currentState == STATE_IDLE) {
                    if (digitalRead(PIN_TTP223) == HIGH) {
                        enterState(STATE_INIT_DELAY);
                        Serial.println(">>> START: INIT_DELAY (3 seconds)");
                    }
                } 
                else if (currentState == STATE_INIT_DELAY) {
                    if (fsm_current_time - state_start_time >= 3000) {
                        float target_angle = getModeAngle(angle_histogram, HIST_SIZE); 
                        
                        if (localData.dist[0] < CONF_ENY) {
                            enterState(STATE_ATK_LOCK);
                            Serial.println(">>> DELAY XONG: ĐỊCH NGAY TRƯỚC MẶT -> LOCK!");
                        } else {
                            enterState(STATE_SEARCH_ENEMY);
                            if (target_angle != 999.0) {
                                xSemaphoreTake(dataMutex, portMAX_DELAY);
                                sysData.enemy_angle = target_angle; 
                                xSemaphoreGive(dataMutex);
                                Serial.print(">>> DELAY XONG: TÌM KIẾM THEO GÓC GHI NHỚ: ");
                                Serial.println(target_angle);
                            } else {
                                Serial.println(">>> DELAY XONG: MÙ HOÀN TOÀN -> VÀO CHẾ ĐỘ QUÉT XOAY ỐC!");
                            }
                        }
                    }
                }
                break;
            }

            // NHÓM 2: PHÒNG THỦ VÀ PHỤC HỒI
            case STATE_DEF_EDGE_AVOID:
            {
                static uint32_t clean_edge_time = 0;
                static int esc_l = -PWM_MAX, esc_r = -PWM_MAX; 

                // Chốt hướng né ngay khoảnh khắc đầu tiên nhảy vào State
                if (state_just_entered) {
                    
                    // Áp dụng lại lớp mặt nạ (Masking) y hệt Core 0
                    bool ignore_front = (localData.pitch > PITCH_TH); 
                    bool ignore_rear  = (localData.pitch < -PITCH_TH);

                    bool edge_FL = (!ignore_front) && (localData.line[0] <= TCRT_EDGE_TH);
                    bool edge_FR = (!ignore_front) && (localData.line[1] <= TCRT_EDGE_TH);
                    bool edge_BL = (!ignore_rear)  && (localData.line[2] <= TCRT_EDGE_TH);
                    bool edge_BR = (!ignore_rear)  && (localData.line[3] <= TCRT_EDGE_TH);

                    // Logic phán đoán không còn bị lừa bởi báo động giả
                    if (edge_FL && edge_FR) { esc_l = -PWM_MAX; esc_r = -PWM_LOW; } // Mũi dính -> Lùi móc cua phải
                    else if (edge_BL && edge_BR) { esc_l = PWM_MAX; esc_r = PWM_LOW; } // Đít dính -> Tiến móc cua phải
                    else if (edge_FL && edge_BR) { esc_l = -PWM_MAX; esc_r = PWM_MAX; } // Chéo 1 -> Xoay tại chỗ lùi
                    else if (edge_FR && edge_BL) { esc_l = PWM_MAX; esc_r = -PWM_MAX; } // Chéo 2 -> Xoay tại chỗ tiến
                    else if (edge_FL) { esc_l = -PWM_MAX; esc_r = -PWM_LOW; } // Trái dính -> Lùi vòng phải
                    else if (edge_FR) { esc_l = -PWM_LOW; esc_r = -PWM_MAX; } // Phải dính -> Lùi vòng trái
                    else if (edge_BL) { esc_l = PWM_MAX; esc_r = PWM_LOW; }   // Đít trái dính -> Tiến vòng phải
                    else if (edge_BR) { esc_l = PWM_LOW; esc_r = PWM_MAX; }   // Đít phải dính -> Tiến vòng trái
                    else { esc_l = -PWM_MAX; esc_r = -PWM_LOW; } // Fallback: Lùi móc cua
                }

                // Liên tục bơm PWM đã chốt
                driveBot(esc_l, esc_r);

                if (!localData.edgeDetect) {
                    if (clean_edge_time == 0) clean_edge_time = fsm_current_time;
                    if (fsm_current_time - clean_edge_time >= EDGE_TIMEOUT) {
                        clean_edge_time = 0;
                        if (localData.dist[0] < WARN_DIST) {
                            enterState(STATE_ATK_STRIKE);
                            Serial.println(">>> SAFE: EDGE CLEARED -> ĐỊCH Ở TRƯỚC MẶT -> STRIKE!");
                        } else {
                            enterState(STATE_SEARCH_ENEMY);
                            Serial.println(">>> SAFE: EDGE CLEARED -> SEARCH");
                        }
                    }
                } else {
                    clean_edge_time = 0;
                }
                break;
            }

            case STATE_DEF_LAST_STAND:
            {
                if (localData.pitch < -PITCH_TH) driveBot(-PWM_MAX, -PWM_MAX); 
                else if (localData.pitch > PITCH_TH) driveBot(PWM_MAX, PWM_MAX);
                else if (localData.roll < -PITCH_TH) driveBot(-PWM_LOW, -PWM_MAX);
                else if (localData.roll > PITCH_TH) driveBot(-PWM_MAX, -PWM_LOW);
                else driveBot(-200, -200); 

                static uint32_t stable_time = 0;
                if (fabsf(localData.pitch) <= 5.0 && fabsf(localData.roll) <= 5.0 && !localData.fallOut) {
                    if (stable_time == 0) stable_time = fsm_current_time;
                    if (fsm_current_time - stable_time >= 100) {
                        enterState(STATE_REC_RECOVER);
                        stable_time = 0;
                        Serial.println(">>> LAST STAND SURVIVED -> RECOVERING");
                    }
                } else {
                    stable_time = 0;
                }
                break;
            }

            case STATE_DEF_ANTI_LIFT:
            {
                uint32_t elapsed_time = fsm_current_time - state_start_time;

                // Nếu bị hếch mũi -> Lùi (-). Nếu bị hếch đít -> Tiến (+)
                int escape_pwm = localData.liftedFront ? -PWM_MAX : PWM_MAX; 
                int jiggle_pwm = localData.liftedFront ? -PWM_JIGGLE : PWM_JIGGLE;

                // CHIẾN THUẬT: Đánh võng hướng ngược lại cái nêm của địch
                if (elapsed_time < 200) {
                    driveBot(escape_pwm, escape_pwm); 
                } 
                else if (elapsed_time < 800) {
                    int phase = ((elapsed_time - 200) / 100) % 2; 
                    if (phase == 0) driveBot(escape_pwm, jiggle_pwm); 
                    else driveBot(jiggle_pwm, escape_pwm);            
                } 
                else {
                    driveBot(escape_pwm * 0.8, escape_pwm * 0.8); // Xả ga đều (200)
                }

                // ĐIỀU KIỆN THOÁT 1: Xe đã hạ xuống sàn thành công
                if (!localData.liftedFront && !localData.liftedRear && fabsf(localData.pitch) <= 5.0 && fabsf(localData.roll) <= 5.0) {
                    enterState(STATE_REC_RECOVER);
                    Serial.println(">>> THOÁT KHỎI GẦM ĐỊCH -> RECOVERING!");
                }
                
                // ĐIỀU KIỆN THOÁT 2 (TIMEOUT)
                else if (elapsed_time > 1200) {
                    enterState(STATE_DEF_LAST_STAND);
                    Serial.println(">>> ANTI_LIFT BẾ TẮC -> LAST STAND!");
                }
                
                break;
            }

            case STATE_DEF_ANTI_PUSH:
            {
                uint32_t elapsed_time = fsm_current_time - state_start_time;

                if (elapsed_time < 100) driveBot(0, 0); // Phanh cứng lại để cản lực đâm ban đầu
                else if (elapsed_time < 300) driveBot(-PWM_MAX, -PWM_MAX); // Giật lùi hết cỡ tạo khoảng cách
                else if (elapsed_time < 600) {
                    // Dao động zíc-zắc có chu kỳ (50ms mỗi pha)
                    int phase = (elapsed_time / 50) % 2; 
                    
                    if (phase == 0) {
                        driveBot(-200, -PWM_PIVOT); // Lắc mạnh đuôi sang trái
                    } else {
                        driveBot(-PWM_PIVOT, -200); // Lắc mạnh đuôi sang phải
                    }
                } 
                else {
                    enterState(STATE_REC_RECOVER);
                    Serial.println(">>> ANTI_PUSH DONE -> RECOVERING");
                }
                break;
            }

            case STATE_DEF_SIDE_GUARD:
            {
                uint32_t elapsed_time = fsm_current_time - state_start_time;
                static int escape_dir = 1;

                // Bắt Entry Action 1 lần duy nhất khi mới vào State
                if (state_just_entered) {
                    escape_dir = (esp_random() & 1) ? 1 : -1;
                }

                if (elapsed_time < 300) driveBot(PWM_MAX * escape_dir, PWM_MAX * escape_dir);
                else {
                    if (localData.dist[0] < WARN_DIST) {
                        enterState(STATE_ATK_LOCK);
                        Serial.println(">>> SIDE_GUARD -> ENEMY FRONT -> ATK_LOCK");
                    } else {
                        enterState(STATE_REC_RECOVER);
                        Serial.println(">>> SIDE_GUARD -> RECOVERING");
                    }
                }
                break;
            }

            case STATE_DEF_REAR_GUARD:
            {
                uint32_t elapsed_time = fsm_current_time - state_start_time;
                static int turn_dir = 1;

                // Xác định hướng lượn vòng dực vào góc bị đâm sau đít
                if (state_just_entered) {
                    // Nếu enemy_angle < 0 (Nửa trái), lượn sang Phải (turn_dir = 1)
                    turn_dir = (localData.enemy_angle < 0) ? 1 : -1;
                }

                if (elapsed_time < 400) {
                    // Lượn vòng: 1 bánh full ga, 1 bánh giảm ga
                    if (turn_dir == 1) driveBot(PWM_MAX, 150); 
                    else driveBot(150, PWM_MAX);
                } 
                else {
                    if (localData.flkPossible) {
                        enterState(STATE_ATK_FLANK_REAR);
                        Serial.println(">>> REAR_GUARD -> TURN TO FLANK REAR");
                    } else {
                        enterState(STATE_REC_RECOVER);
                        Serial.println(">>> REAR_GUARD -> RECOVERING");
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
                     search_dir = (localData.enemy_angle < 0) ? -1 : 1;
                }

                if (elapsed_time < 1000) {
                    driveBot(150 * search_dir, -150 * search_dir); // Xoay tại chỗ quét nhanh
                } 
                else if (elapsed_time < 3000) {
                    driveBot(200 * search_dir, 50 * search_dir);  // Quét vòng cung rộng
                } 
                else {
                    search_dir = -search_dir; 
                    state_start_time = fsm_current_time; // Bắt đầu chu kỳ tìm kiếm mới
                    Serial.print(">>> SEARCH TIMEOUT -> REVERSE DIRECTION (Dir: ");
                    Serial.print(search_dir);
                    Serial.println(")");
                }

                // Tìm thấy mục tiêu
                if (localData.dist[0] < CONF_ENY) {
                    // Cự ly nguy hiểm -> Bỏ qua ngắm nghía, ATK
                    if (localData.dist[0] <= WARN_DIST) {
                        enterState(STATE_ATK_STRIKE);
                        Serial.println(">>> SEARCH: ĐỊCH Ở GẦN (< WARN_DIST) -> RUSH LUÔN BỎ QUA LOCK!");
                    } else {
                        enterState(STATE_ATK_LOCK);
                        Serial.println(">>> SEARCH: FOUND IN FRONT (FAR) -> LOCK");
                    }
                }
                else if (localData.dist[1] < CONF_ENY || localData.dist[2] < CONF_ENY || 
                         localData.dist[3] < CONF_ENY || localData.dist[4] < CONF_ENY) {
                    enterState(STATE_ATK_FLANK_SIDE);
                    Serial.println(">>> SEARCH: FOUND AT SIDE -> FLANK");
                }
                break;
            }

            case STATE_REC_RECOVER:
            {
                static int random_turn_dir = 1; 
                uint32_t elapsed_time = fsm_current_time - state_start_time;

                // Bắt Entry Action
                if (state_just_entered) {
                    random_turn_dir = (esp_random() & 1) ? 1 : -1;
                }

                if (elapsed_time < 300) driveBot(-200, -200);
                else if (elapsed_time < MAX_RECOVER_TIME) driveBot(150 * random_turn_dir, -150 * random_turn_dir);

                if (elapsed_time >= 300) {
                    if (localData.dist[0] < WARN_DIST) {
                        enterState(STATE_ATK_LOCK);
                        Serial.println(">>> RECOVER INT: ENEMY FRONT -> ATK_LOCK");
                    } 
                    else if (localData.dist[1] < WARN_DIST || localData.dist[2] < WARN_DIST || 
                             localData.dist[3] < WARN_DIST || localData.dist[4] < WARN_DIST) {
                        enterState(STATE_ATK_FLANK_SIDE);
                        Serial.println(">>> RECOVER INT: ENEMY SIDE -> FLANK_SIDE");
                    }
                }

                if (elapsed_time >= MAX_RECOVER_TIME) {
                    enterState(STATE_SEARCH_ENEMY);
                    Serial.println(">>> RECOVER TIMEOUT -> SEARCH");
                }
                break;
            }

            // NHÓM 3: TẤN CÔNG (ATTACK)
            case STATE_ATK_LOCK:
            {
                static uint8_t lock_retries = 0;
                uint32_t elapsed_time = fsm_current_time - state_start_time;

                if (state_just_entered) stalemate_cycles = 0;

                // AN TOÀN: Mất mục tiêu hoàn toàn khỏi 5 mắt (Khoảng mù)
                if (localData.dist[0] > CONF_ENY && localData.dist[1] > CONF_ENY && 
                    localData.dist[2] > CONF_ENY && localData.dist[3] > CONF_ENY && 
                    localData.dist[4] > CONF_ENY) {
                    
                    lock_retries++;
                    driveBot(0, 0); 

                    if (lock_retries >= MAX_LOCK_RETRIES) {
                        enterState(STATE_SEARCH_ENEMY);
                        lock_retries = 0;
                        Serial.println(">>> LOCK FAILED: LOST TARGET -> SEARCH");
                    } else {
                        state_start_time = fsm_current_time; // Reset timer để thử ngắm lại
                    }
                    break;
                }

                // KẾT VÀO: Lái xe (Steering) hướng mục tiêu vào chính diện
                float err_angle = localData.enemy_angle; 
                
                // Triệt tiêu độ giật (Discontinuity) khi góc dao động quanh ngưỡng TIGHT
                int forward_pwm = 0;
                int turn_pwm = 0;

                if (fabsf(err_angle) <= ANGLE_TIGHT) {
                    driveBot(PWM_HIGH, PWM_HIGH); // Thẳng tắp -> Phóng thẳng
                } else {
                    // Chỉ tính turn_pwm khi ngoài vùng ANGLE_TIGHT
                    turn_pwm = constrain(fabsf(err_angle) * KP_STEERING, PWM_TURN_MIN, PWM_HIGH); 
                    
                    if (fabsf(err_angle) < ANGLE_FLANK) { 
                        forward_pwm = PWM_MED; // Vẫn đang nhìn thấy khá rõ -> Vừa tiến vừa bẻ
                    } else {
                        forward_pwm = 0; // Lệch góc quá gắt -> Xoay tại chỗ (Pivot) để bắt hình nhanh
                    }

                    if (err_angle > 0) {
                        driveBot(forward_pwm + turn_pwm, forward_pwm - turn_pwm);
                    } else {
                        driveBot(forward_pwm - turn_pwm, forward_pwm + turn_pwm);
                    }
                }

                // ĐIỀU KIỆN KÍCH HOẠT LỰC ĐÁNH (STRIKE/FLANK/RUSH)
                bool is_ready_to_strike = false;
                if (localData.dist[0] < WARN_DIST && fabsf(err_angle) <= ANGLE_WIDE) {
                    is_ready_to_strike = true;
                } 
                else if (localData.dist[0] < DIST_CLOSE && fabsf(err_angle) <= ANGLE_SLOPPY) {
                    is_ready_to_strike = true; // Gần sát rồi, lệch góc cũng ủi luôn
                }

                if (is_ready_to_strike) {
                    if (!localData.sideDanger) {
                        // Cây quyết định (Decision Tree)
                        if (localData.closingFast) {
                            enterState(STATE_ATK_DELAY_RUSH);
                            lock_retries = 0;
                            Serial.println(">>> LOCK SUCCESS -> ENEMY RUSHING -> DELAY RUSH!");
                        } 
                        else {
                            if (localData.dist[0] > DIST_CLOSE && (esp_random() % 100 < FEINT_CHANCE)) {
                                enterState(STATE_ATK_FEINT);
                                lock_retries = 0;
                                Serial.println(">>> LOCK SUCCESS -> TACTICAL FEINT!");
                            } else {
                                enterState(STATE_ATK_STRIKE);
                                lock_retries = 0; 
                                Serial.println(">>> LOCK SUCCESS -> DEFAULT STRIKE!");
                            }
                        }
                    }
                }
                
                // KIỂM SOÁT TIMEOUT CỰC ĐOAN (Bế tắc vật lý)
                if (elapsed_time > ATK_LOCK_TIME) {
                    if (localData.dist[0] < CONF_ENY) {
                        enterState(STATE_ATK_STRIKE);
                        Serial.println(">>> LOCK TIMEOUT -> DESPERATE STRIKE");
                    } else {
                        enterState(STATE_SEARCH_ENEMY);
                        Serial.println(">>> LOCK TIMEOUT -> MẤT DẤU -> SEARCH");
                    }
                    lock_retries = 0;
                }
                
                break;
            }

            case STATE_ATK_STRIKE:
            {
                uint32_t elapsed_time = fsm_current_time - state_start_time;

                // CHUỖI RA ĐÒN: Bơm xung động lượng (Momentum)
                if (elapsed_time < PUSH_MS) {
                    driveBot(PWM_MAX, PWM_MAX); // Xung lực tối đa (Bắn tốc)
                } 
                else if (elapsed_time < PUSH_MS + HOLD_PUSH_MS) {
                    driveBot(220, 220); // Duy trì áp lực (Đẩy sáp lá cà)
                } 
                else {
                    driveBot(200, 200); // Theo xe, tránh trượt bánh (Wheel slip)
                }

                // CHECK PHẢN CÔNG (ANTI-PUSH) TỪ ĐỐI THỦ
                // Đủ mù (IGNORE_ANTI_PUSH) để không tự kích hoạt do gia tốc của chính mình
                if (elapsed_time > IGNORE_ANTI_PUSH) {
                    if (localData.sideDanger == true && localData.impactDetected == true) {
                        enterState(STATE_DEF_ANTI_PUSH); 
                        Serial.println(">>> STRIKE BLOCKED: ANTI_PUSH TRIGGERED!");
                        break; 
                    }
                }

                if (localData.liftDetected) {
                    enterState(STATE_ATK_LIFT);
                    Serial.println(">>> STRIKE -> ENEMY LIFTED -> ATK_LIFT (FULL POWER)!");
                    break;
                }

                // ĐIỀU KIỆN THOÁT (HỤT ĐÒN HOẶC MẤT GÓC)
                // - Hụt mục tiêu do đối thủ lùi nhanh hơn hoặc bị hất văng: dist > WARN_DIST
                // - Lệch góc quá nhiều: angle > 20 độ
                if (localData.dist[0] > WARN_DIST || fabsf(localData.enemy_angle) > 20.0) {
                    enterState(STATE_ATK_LOCK); 
                    Serial.println(">>> STRIKE SLIP/AWAY -> RE-LOCK");
                    break;
                }

                // IỂM SOÁT BẾ TẮC (STALEMATE) THAY VÌ "DRIVE FOREVER"
                // Nếu hai xe đang húc nhau giằng co quá lâu (vượt TIMEOUT_MAX),
                // Việc ủi thẳng mãi sẽ làm cháy động cơ/tuột bánh răng.
                // Giải pháp: Kích hoạt tự hãm của Worm Gear bằng cách phanh cứng 0 PWM.
                if (elapsed_time > TIMEOUT_MAX) {
                enterState(STATE_ATK_STALEMATE_BRAKE);
                Serial.println(">>> STALEMATE TIMEOUT -> WORM GEAR BRAKE (STAND YOUR GROUND)!");
                }

        break;
      } 
            case STATE_ATK_STALEMATE_BRAKE:
            {
                uint32_t elapsed_time = fsm_current_time - state_start_time;

                // Ép PWM = 0 để khóa cứng bánh răng (Only for Worm Gear)
                driveBot(0, 0);

                // Đứng im trong 250ms 
                if (elapsed_time >= 250) {
                    if (localData.dist[0] <= WARN_DIST && fabsf(localData.enemy_angle) <= 15.0) {
                        
                        // Địch vẫn lù lù trước mặt -> Tăng biến đếm bế tắc
                        stalemate_cycles++; 
                        
                        if (stalemate_cycles >= 5) {
                            // Đã húc nhau 5 nhịp (hơn 6 giây) không kết quả -> PHÁ THẾ BẾ TẮC
                            stalemate_cycles = 0;
                            enterState(STATE_REC_RECOVER); // Giật lùi nhanh và xoay tìm góc đánh sườn
                            Serial.println(">>> STALEMATE BROKEN -> FALLBACK TO RECOVER!");
                        } else {
                            // Mới húc nhịp đầu, bồi thêm nhát STRIKE nữa xem sao!
                            enterState(STATE_ATK_STRIKE);
                            Serial.println(">>> BRAKE DONE -> RE-STRIKE!");
                        }
                    } else {
                        // Địch bị trượt hoặc lùi lại -> Nhìn lại góc cho chuẩn
                        stalemate_cycles = 0; // Reset bộ đếm
                        enterState(STATE_ATK_LOCK);
                        Serial.println(">>> BRAKE DONE -> ENEMY SLIPPED -> RE-LOCK");
                    }
                }
                break;
            }
            case STATE_ATK_FLANK_FRONT:
            {
                
                float err_angle = localData.enemy_angle;
                int base_pwm = 200; 
                int offset_pwm = 50; 

                if (err_angle < -5.0) driveBot(base_pwm - offset_pwm, base_pwm + offset_pwm); 
                else if (err_angle > 5.0) driveBot(base_pwm + offset_pwm, base_pwm - offset_pwm);
                else driveBot(base_pwm, base_pwm);

                if (localData.impactDetected) {
                    enterState(STATE_ATK_STRIKE);
                    Serial.println(">>> FLANK_FRONT IMPACT -> GO TO STRIKE");
                }
                else if (fabsf(err_angle) >= 25.0) {
                    enterState(STATE_ATK_FLANK_SIDE);
                    Serial.println(">>> FLANK_FRONT SLIP -> GO TO FLANK_SIDE");
                }
                else if (localData.dist[0] > CONF_ENY) {
                    enterState(STATE_SEARCH_ENEMY);
                    Serial.println(">>> FLANK_FRONT LOST TARGET -> SEARCH");
                }
                break;
            }

            case STATE_ATK_FLANK_SIDE:
            {
                uint16_t min_left = min(localData.dist[1], localData.dist[3]);
                uint16_t min_right = min(localData.dist[2], localData.dist[4]);
                
                int thrust_pwm = PWM_MAX;
                int pivot_pwm = -PWM_PIVOT; // Mặc định tạt sườn từ xa là lượn vòng cung (Arc)

                // Bẻ cua cực gắt (Pivot) nếu địch áp sát sườn hoặc sát mũi
                if (localData.dist[0] < 200 || min_left < 150 || min_right < 150) {
                    pivot_pwm = -PWM_MAX; // Bánh trong giật lùi full tốc độ
                    thrust_pwm = PWM_MAX; // Bánh ngoài tiến full tốc độ -> Xoay tại chỗ (Spin on a dime)
                }

                bool turn_left = false;

                // Trường hợp mù tạm thời (chỉ dựa vào IMU vector)
                if (min_left >= CONF_ENY && min_right >= CONF_ENY) {
                    turn_left = (localData.enemy_angle < 0);
                } 
                // Chống nhiễu ToF: Chênh lệch dưới 20mm coi như địch áp sát đều 2 bên
                else if (fabsf((int)min_left - (int)min_right) <= 20) {
                    // Cài cắm sự khó lường: Tung đồng xu!
                    turn_left = (esp_random() & 1); 
                } 
                // Đánh vào điểm yếu: Bên nào khoảng cách nhỏ hơn (địch hở sườn nhiều hơn) thì lách vào bên đó!
                else if (min_left < min_right) {
                    turn_left = true;
                } else {
                    turn_left = false;
                }

                // Thi hành mệnh lệnh
                if (turn_left) driveBot(pivot_pwm, thrust_pwm);
                else driveBot(thrust_pwm, pivot_pwm);

                if (localData.impactDetected) {
                    enterState(STATE_ATK_STRIKE);
                    Serial.println(">>> FLANK_SIDE IMPACT -> GO TO STRIKE");
                }
                else if (localData.dist[0] < CONF_ENY) {
                    // Đang tạt sườn mà địch lọt vào mũi xe ở cự ly nguy hiểm -> ATK
                    if (localData.dist[0] <= WARN_DIST) { 
                        enterState(STATE_ATK_STRIKE);
                        Serial.println(">>> FLANK_SIDE: TARGET IN FRONT (< WARN_DIST) -> RUSH TO STRIKE!");
                    } else {
                        // Địch lọt vào mũi nhưng còn ở xa -> SLOW-DOWN 2 LOCK
                        enterState(STATE_ATK_LOCK);
                        Serial.println(">>> FLANK_SIDE: TARGET FAR IN FRONT -> RE-LOCK");
                    }
                }
                else if (localData.dist[0] > CONF_ENY && localData.dist[1] > CONF_ENY && 
                         localData.dist[2] > CONF_ENY && localData.dist[3] > CONF_ENY && 
                         localData.dist[4] > CONF_ENY) {
                    
                    if (fsm_current_time - state_start_time > 800) { 
                        enterState(STATE_REC_RECOVER);
                        Serial.println(">>> FLANK_SIDE FAILED -> RECOVERING");
                    }
                }
                break;
            }

            case STATE_ATK_FLANK_REAR:
            {
                
                bool enemy_on_left = (localData.dist[3] < CONF_ENY) || (localData.enemy_angle <= -120.0);

                if (fabsf(localData.v_e) < 100.0) { 
                    if (enemy_on_left) driveBot(PWM_PIVOT, PWM_MAX); 
                    else driveBot(PWM_MAX, PWM_PIVOT); 
                } else {
                    if (enemy_on_left) driveBot(-PWM_MAX, PWM_MAX); 
                    else driveBot(PWM_MAX, -PWM_MAX); 
                }

                if (localData.impactDetected) {
                    enterState(STATE_ATK_STRIKE);
                    Serial.println(">>> FLANK_REAR IMPACT -> GO TO STRIKE");
                }
                else if (localData.dist[0] < CONF_ENY) {
                    enterState(STATE_ATK_LOCK); 
                    Serial.println(">>> FLANK_REAR: TARGET IN FRONT -> GO TO ATK_LOCK");
                }
                else if (localData.dist[0] > CONF_ENY && localData.dist[1] > CONF_ENY && 
                         localData.dist[2] > CONF_ENY && localData.dist[3] > CONF_ENY && 
                         localData.dist[4] > CONF_ENY) {
                    
                    if (fsm_current_time - state_start_time > 600) {
                        enterState(STATE_SEARCH_ENEMY);
                        Serial.println(">>> FLANK_REAR: LOST TARGET -> SEARCH");
                    }
                }
                break;
            }

            case STATE_ATK_LIFT:
            {
                uint32_t elapsed_time = fsm_current_time - state_start_time; // Thêm bộ đếm thời gian
                driveBot(PWM_MAX, PWM_MAX);
                
                // Thêm điều kiện: Nếu nhấc đối thủ quá 1.5s (4400ms) thì tự động buông để chống kẹt
                if (!localData.liftDetected || elapsed_time > 4400) { 
                    if (localData.dist[0] < WARN_DIST) enterState(STATE_ATK_STRIKE);
                    else enterState(STATE_SEARCH_ENEMY);
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
                    enterState(STATE_ATK_FLANK_SIDE); 
                    Serial.println(">>> FEINT COMPLETE -> GO TO FLANK_SIDE");
                }
                if (localData.impactDetected) {
                    enterState(STATE_ATK_STRIKE);
                    Serial.println(">>> FEINT IMPACT EARLY -> STRIKE");
                }
                if (localData.dist[0] > CONF_ENY && elapsed_time > 100) {
                    enterState(STATE_SEARCH_ENEMY);
                    Serial.println(">>> FEINT MISSED -> SEARCH");
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

                // CHỜ KHOẢNG CÁCH, KHÔNG CHỜ THỜI GIAN
                // Ngay khi địch lọt vào "vùng tử thần" (< 150mm), dậm kịch ga để bẩy nó lên!
                if (localData.dist[0] <= 150) {
                    enterState(STATE_ATK_STRIKE); 
                    Serial.println(">>> COUNTER RUSH: ĐỊCH VÀO TẦM (<150mm) -> BUNG MAX GA (UPPERCUT)!");
                }
                // Nếu địch lươn lẹo bẻ lái sang hướng khác (lệch góc > 20 độ) -> Hủy rình, ngắm lại
                else if (fabsf(localData.enemy_angle) > 20.0) {
                    enterState(STATE_ATK_LOCK); 
                    Serial.println(">>> COUNTER CANCELLED: ĐỊCH LÁCH GÓC -> RE-LOCK");
                }
                // Nếu địch đột ngột nhát gan phanh lại hoặc đi chậm (v_e <= 100) -> Bỏ rình, chủ động lao lên atk
                else if (localData.v_e <= 100.0) { 
                    enterState(STATE_ATK_STRIKE); 
                    Serial.println(">>> COUNTER CANCELLED: ĐỊCH CHẬM LẠI -> CHỦ ĐỘNG STRIKE");
                }
                // Bế tắc thời gian (Đề phòng 2 xe gằm ghè nhau ngoài tầm 150mm quá lâu)
                else if (elapsed_time > 600) {
                    enterState(STATE_ATK_STRIKE);
                    Serial.println(">>> COUNTER TIMEOUT -> ÉP XUNG ĐÁNH BỪA");
                }
                
                break;
            }

            default:
                enterState(STATE_IDLE);
                driveBot(0, 0);
                break;
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
    RobotState oldState = currentState; // Giữ lại state cũ để so sánh
    previousState = oldState;

    currentState = newState;
    state_start_time = millis(); 
    state_just_entered = true;

    // --- LOGIC OLED ---
    if (oldState == STATE_INIT_DELAY) {
        go_lock = true;
        go_start_time = millis();
        needsDisplayUpdate = true; // Chỉ bật cờ, KHÔNG VẼ Ở ĐÂY
    } else {
        if (newState == STATE_DEF_LAST_STAND) {
            go_lock = false;
            needsDisplayUpdate = true; // Chuyển việc vẽ xuống loop()
        } 
        else if (!go_lock) { 
            // KHÔNG chặn IDLE và INIT_DELAY nữa
            needsDisplayUpdate = true;
        }
    }
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
        case STATE_ATK_FLANK_FRONT: return "FLANK_FRONT";
        case STATE_ATK_FLANK_SIDE: return "FLANK_SIDE";
        case STATE_ATK_FLANK_REAR: return "FLANK_REAR";
        case STATE_ATK_LIFT: return "ATK_LIFT";
        case STATE_ATK_FEINT: return "FEINT";
        case STATE_ATK_DELAY_RUSH: return "DELAY_RUSH";
        case STATE_ATK_LOCK: return "LOCK";
        case STATE_ATK_STALEMATE_BRAKE: return "BRAKE";
        case STATE_DEF_ANTI_PUSH: return "ANTI_PUSH";
        case STATE_DEF_SIDE_GUARD: return "SIDE_GUARD";
        case STATE_DEF_REAR_GUARD: return "REAR_GUARD";
        case STATE_DEF_EDGE_AVOID: return "EDGE_AVOID";
        case STATE_DEF_ANTI_LIFT: return "ANTI_LIFT";
        case STATE_DEF_LAST_STAND: return "LAST_STAND";
        case STATE_REC_RECOVER: return "RECOVER";
        case STATE_SEARCH_ENEMY: return "SEARCH";
        default: return "UNKNOWN";
    }
}
