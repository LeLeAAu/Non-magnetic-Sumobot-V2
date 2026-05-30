/*
- Core 0: Chạy TaskSensor (Đọc ToF, IMU, TCRT, tính toán Kinematics)
- Core 1: Chạy TaskFSM (Ra quyết định trạng thái) và Loop (Cập nhật OLED/Debug)
- Giao tiếp liên tác vụ thông qua Mutex bảo vệ cấu trúc SystemData
*/

// Việc dùng ToF tạo khoảng delay đo khoảng cách khá lớn. Nếu sau này có một cuộc thi khác thì sẽ chuyển dùng cảm biến ánh sáng khác

// KHÔNG ĐƯỢC GIẢM BUDGET setMeasurementTimingBudget(33000) XUỐNG VÌ SẼ GÂY RA LỖI

// KHÔNG DÙNG CÁC FILTER LỌC NHIỄU GÌ ĐÓ BỞI VÌ ĐO BẰNG VLX CÓ DELAY RẤT LỚN, NẾU DÙNG NÓ CÓ THỂ LÀM GIẢM KHẢ NĂNG PHẢN ỨNG CỦA BOT

//Todo
/*



-
*/


// Qs - Ans
/*
- Có nên giảm vTaskDelay(pdMS_TO_TICKS(50)); xuống k? - Không

*/

// CMD: pip install pyserial numpy matplotlib

#include "Config.h"
#include "MotorControl.h"
#include "DisplayFace.h"
#include "SensorTask.h"
#include "FSMTask.h"
#include <atomic>
#include "telemetry_utils.h"
#include <freertos/semphr.h>

#if SIMULATION_MODE
volatile uint16_t sim_in_dist[5] = {2000, 2000, 2000, 2000, 2000};
volatile uint16_t sim_in_line[5] = {4095, 4095, 4095, 4095, 4095};
volatile uint16_t sim_in_ttp223 = 0;
#endif

SemaphoreHandle_t paramMutex;

// Khởi tạo đối tượng hệ thống
TwoWire I2COLED = TwoWire(1);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &I2COLED, OLED_RESET);

// Quản lí dữ liệu và trạng thái toàn cục
SystemData sysBuffer[2];               // Double-Buffer // thay thế cho sysData đơn
std::atomic<uint8_t> read_index(0);    // Con trỏ nguyên tử chỉ vị trí buffer FSM sẽ đọc
std::atomic<uint8_t> active_pwm(0);       // Tách riêng PWM ra biến atomic để tránh ghi đè chéo
volatile RobotState currentState = STATE_IDLE;
volatile RobotState previousState = STATE_IDLE;
volatile bool needsDisplayUpdate = false;

// Biến điều khiển nguồn logic
bool go_lock = false;   // Cờ hiển thị chữ GO! sau đếm ngược
uint32_t go_start_time = 0; 
uint32_t state_start_time = 0; // Lưu mốc thời gian bắt đầu của mỗi trạng thái FSM
bool state_just_entered = false;

// Quản lí TASKS
TaskHandle_t TaskSensorHandle = NULL;
TaskHandle_t TaskFSMHandle = NULL;

// Config cảm biến
VL53L1X sensorsToF[5]; 
LSM6DS3 myIMU(I2C_MODE, 0x6B); 

void setup() {
    Serial.begin(115200);
    paramMutex = xSemaphoreCreateMutex();
    // Khởi tạo OLED sớm để hiện thị Boot
    I2COLED.begin(OLED_SDA, OLED_SCL, 100000); // Khởi tạo I2C1
    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        DEBUG_PRINTLN(F("OLED fail"));
    } else {
        showLoading(); // Animation khởi động
        display.clearDisplay();
        display.display();
    }

    // Cấu hình bus cảm biến chính
    Wire.begin(I2C_SDA, I2C_SCL); // <--- ÉP I2C CHẠY TRÊN CHÂN 26 VÀ 25
    Wire.setTimeOut(10); // Ép phần cứng I2C tự nhả luồng sau 10ms nếu bị treo
    Wire.setClock(400000); // Fast mode I2C

    bool hardwareError = false;

    // Cấu hình I/0 cho cảm biến TCRT (Dò line và bụng)
    pinMode(PIN_TCRT_FL, INPUT);
    pinMode(PIN_TCRT_FR, INPUT);
    pinMode(PIN_TCRT_BL, INPUT);
    pinMode(PIN_TCRT_BR, INPUT);
    pinMode(PIN_TCRT_DETECT, INPUT);
    // Cấu hình I/0 cho cảm biến chạm TTP223
    pinMode(PIN_TTP223, INPUT_PULLDOWN);

    // Thiết lập PWM cho Motor
    ledcAttach(PIN_MOTOR_L_RPWM, 20000, 8);
    ledcAttach(PIN_MOTOR_L_LPWM, 20000, 8);
    ledcAttach(PIN_MOTOR_R_RPWM, 20000, 8);
    ledcAttach(PIN_MOTOR_R_LPWM, 20000, 8);
#if !SIMULATION_MODE
    // Khởi tạo IMU
    if (myIMU.begin() != 0) {
        DEBUG_PRINTLN("IMU Error!");
        hardwareError = true;
    }

    // Gán địa chỉ động (Sequential XSHUT) cho 5 ToF
    for (int i = 0; i < 5; i++) {
        pinMode(XSHUT_PINS[i], OUTPUT);
        digitalWrite(XSHUT_PINS[i], LOW);
    }
    delay(10);

    for (int i = 0; i < 5; i++) {
        digitalWrite(XSHUT_PINS[i], HIGH);
        delay(10);
        sensorsToF[i].setTimeout(50);
        if (!sensorsToF[i].init()) {
            Serial.print("Lỗi khởi tạo VL53L1X số "); DEBUG_PRINTLN(i);
            hardwareError = true;
        } else {
            sensorsToF[i].setAddress(VLX_ADDRESSES[i]); 
            sensorsToF[i].setDistanceMode(VL53L1X::Long); // Chế độ đo xa 4m, chế độ đo gần không thể dùng trong thực tế do nó gặp nhiều lỗi làm cho việc tính toán lỗi theo
            sensorsToF[i].setMeasurementTimingBudget(33000); // 33ms/mẫu
            sensorsToF[i].startContinuous(34); // Min là 33, để 34 để tối ưu hoá
        }
    }

    // Nếu lỗi phần cứng nghiêm trọng, dừng hệ thống và báo X_X
    if (hardwareError) {
        display.clearDisplay();
        display.setTextSize(4);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(28, 0);
        display.print("X_X");
        display.display();
        
        DEBUG_PRINTLN(">>> HARDWARE HALT: SENSOR FAILED! BOOT ABORTED. <<<");
        // Khóa cứng hệ thống ở đây, nháy LED hoặc chờ kỹ sư can thiệp
        while(true) {
            delay(100);
        }
    }
#else
    DEBUG_PRINTLN(">>> SIMULATION MODE: Bypassing hardware init");
#endif
    DEBUG_PRINTLN("Init phan cung xong!");

    // Kích hoạt đa nhiệm, đẩy các Task vào các Core tương ứng
    xTaskCreatePinnedToCore(TaskSensorCode, "TaskSensor", 10000, NULL, 1, &TaskSensorHandle, 0);
    xTaskCreatePinnedToCore(TaskFSMCode, "TaskFSM", 10000, NULL, 3, &TaskFSMHandle, 1);
#if DEBUG_LEVEL >= 1
    xTaskCreatePinnedToCore(TelemetryRxTask, "TelemetryRx", 4096, NULL, 1, NULL, 1);
#endif
}

void TelemetryRxTask(void *pvParameters) {
    uint8_t rx_buffer[256];
    uint8_t decode_buffer[256];
    size_t rx_idx = 0;
    bool in_packet = false;

    while (1) {
        while (Serial.available()) {
            uint8_t c = Serial.read();
            if (!in_packet && c == 0x00) {
                in_packet = true;
                rx_idx = 0;
            } else if (in_packet && c == 0x00) {
                // Kết thúc gói
                if (rx_idx > 0) {
                    int dec_len = cobs_decode(rx_buffer, rx_idx, decode_buffer);
                    if (dec_len >= 3) { // type + length + crc
                        uint8_t type = decode_buffer[0];
                        uint16_t len = decode_buffer[1] | (decode_buffer[2] << 8);
                        if (dec_len >= 3 + len + 2) {
                            uint16_t recv_crc = decode_buffer[3+len] | (decode_buffer[3+len+1] << 8);
                            uint16_t calc_crc = crc16_ccitt(decode_buffer, 3+len);
                            if (recv_crc == calc_crc) {
                                if (type == 0x02) { // PARAM_SET
                                    // payload: param_id (2 byte) + value (4 byte float)
                                    if (len >= 6) {
                                        uint16_t param_id = decode_buffer[3] | (decode_buffer[4] << 8);
                                        float value;
                                        memcpy(&value, &decode_buffer[5], 4);
                                        updateParameter(param_id, value);
                                    }
                                } else if (type == 0x03) { // PARAM_GET
                                    sendAllParameters();
                                } else if (type == 0x05) { // SIM_INJECT_DATA
#if SIMULATION_MODE
                                    if (len >= 20) { // 10 biến uint16 = 20 bytes
                                        memcpy((void*)sim_in_dist, &decode_buffer[3], 10);
                                        memcpy((void*)sim_in_line, &decode_buffer[13], 10);
                                        memcpy((void*)&sim_in_ttp223, &decode_buffer[23], 2); // Trích xuất trạng thái TTP223
                                    }
#endif
                                }
                            }
                        }
                    }
                }
                in_packet = false;
            } else if (in_packet && rx_idx < sizeof(rx_buffer)) {
                rx_buffer[rx_idx++] = c;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}
void updateParameter(uint16_t param_id, float value) {
    if (xSemaphoreTake(paramMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        switch (param_id) {
            case 1:  WARN_DIST = (uint16_t)value; break;
            case 2:  STRIKE_DIST = (uint16_t)value; break;
            case 3:  PWM_MAX = (uint8_t)value; break;
            case 4:  PWM_STRIKE_HOLD = (uint8_t)value; break;
            case 5:  PWM_HIGH = (uint8_t)value; break;
            case 6:  PWM_MED = (uint8_t)value; break;
            case 7:  PWM_LOW = (uint8_t)value; break;
            case 8:  V_MAX_60 = value; break;
            case 9:  OMEGA_60 = value; break;
            case 10: KP_STEERING = value; break;
            case 12: ATK_LOCK_TIME = (uint32_t)value; break;
            case 13: TCRT_EDGE_TH = (uint16_t)value; break;
            case 14: TOF_OFFSET[0] = (int16_t)value; break;
            case 15: TOF_OFFSET[1] = (int16_t)value; break;
            case 16: TOF_OFFSET[2] = (int16_t)value; break;
            case 17: TOF_OFFSET[3] = (int16_t)value; break;
            case 18: TOF_OFFSET[4] = (int16_t)value; break;
            default: break;
        }
        xSemaphoreGive(paramMutex);
        DEBUG_PRINTF(">>> PARAM ALTERED: ID %d -> %.2f\n", param_id, value);
    }
}
void sendAllParameters() {
    uint8_t payload[11 * 6]; // 11 tham số, mỗi tham số gồm 2 byte ID + 4 byte Float
    int idx = 0;

    // Biểu thức Lambda hỗ trợ đóng gói nhanh dữ liệu
    auto packParam = [&](uint16_t id, float val) {
        payload[idx++] = id & 0xFF;
        payload[idx++] = (id >> 8) & 0xFF;
        memcpy(&payload[idx], &val, 4);
        idx += 4;
    };

    if (xSemaphoreTake(paramMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        packParam(1,  (float)WARN_DIST);
        packParam(2,  (float)STRIKE_DIST);
        packParam(3,  (float)PWM_MAX);
        packParam(4,  (float)PWM_STRIKE_HOLD);
        packParam(5,  (float)PWM_HIGH);
        packParam(6,  (float)PWM_MED);
        packParam(7,  (float)PWM_LOW);
        packParam(8,  V_MAX_60);
        packParam(9,  OMEGA_60);
        packParam(10, KP_STEERING);
        packParam(12, (float)ATK_LOCK_TIME);
        packParam(13, (float)TCRT_EDGE_TH);
        packParam(14, (float)TOF_OFFSET[0]);
        packParam(15, (float)TOF_OFFSET[1]);
        packParam(16, (float)TOF_OFFSET[2]);
        packParam(17, (float)TOF_OFFSET[3]);
        packParam(18, (float)TOF_OFFSET[4]);
        xSemaphoreGive(paramMutex);
    }

    // Đóng gói Frame theo đúng định dạng Python mong đợi (Type 0x04)
    uint8_t frame[256];
    frame[0] = 0x04; // PARAM_RESP_TYPE
    frame[1] = idx & 0xFF;
    frame[2] = (idx >> 8) & 0xFF;
    memcpy(frame + 3, payload, idx);

    uint16_t crc = crc16_ccitt(frame, 3 + idx);
    frame[3 + idx] = crc & 0xFF;
    frame[3 + idx + 1] = (crc >> 8) & 0xFF;

    uint8_t cobs_buf[256];
    size_t cobs_len = cobs_encode(frame, 3 + idx + 2, cobs_buf);

    // Xuất luồng nhị phân ra cổng Serial
    Serial.write(0x00);
    Serial.write(cobs_buf, cobs_len);
    Serial.write(0x00);
}

#if DEBUG_LEVEL >= 2
void printDebugInfo(const SystemData& snap) {
    DEBUG_PRINTLN("================================================================");
    
    Serial.print("[FSM] STATE: ");
    Serial.print(getStateName(currentState));
    Serial.print(" | TimeInState: ");
    Serial.print(millis() - state_start_time);
    Serial.print(" ms | PWM Output: ");
    DEBUG_PRINTLN(snap.current_PWM);

    Serial.print("[ToF] Dist: ");
    for(int i=0; i<5; i++) { 
        Serial.print(snap.dist[i]); Serial.print(" "); 
    }
    Serial.print(" | Target: ");
    if (snap.isTargetLost) {
        DEBUG_PRINTLN("LOST");
    } else {
        Serial.print(snap.enemy_angle, 1);
        Serial.print(" deg | v_e: ");
        Serial.print(snap.v_e, 1);
        DEBUG_PRINTLN(" mm/s");
    }

    Serial.print("[TCRT] Line: ");
    for(int i=0; i<4; i++) { 
        Serial.print(snap.line[i]); Serial.print(" "); 
    }
    Serial.print("| Bụng: ");
    DEBUG_PRINTLN(analogRead(PIN_TCRT_DETECT));

    Serial.print("[IMU] P: ");
    Serial.print(snap.pitch, 1);
    Serial.print(" | R: ");
    Serial.print(snap.roll, 1);
    Serial.print(" | [FLAGS]: ");

    if(!snap.edgeDetect && !snap.fallOut && !snap.liftedFront && !snap.liftedRear && 
       !snap.beingLifted && !snap.impactDetected && !snap.closingFast && 
       !snap.sideDanger && !snap.flkPossible) {
        Serial.print("ALL CLEAR");
    } else {
        if(snap.edgeDetect) Serial.print("EDGE! ");
        if(snap.fallOut) Serial.print("FALL! ");
        if(snap.liftedFront) Serial.print("LIFT_F! ");
        if(snap.liftedRear) Serial.print("LIFT_R! ");
        if(snap.beingLifted) Serial.print("BEING_LIFTED! ");
        if(snap.impactDetected) Serial.print("IMPACT! ");
        if(snap.closingFast) Serial.print("RUSHING! ");
        if(snap.sideDanger) Serial.print("SIDE_DANGER! ");
        if(snap.flkPossible) Serial.print("FLANK_READY ");
    }
    DEBUG_PRINTLN("\n");
}
#endif

void loop() {
    uint32_t current_time = millis();

    // Quản lí hiển thị trên OLED
    // Cập nhật giao diện dựa trên trạng thái hiện tại của bot
    if (go_lock && (current_time - go_start_time >= 500)) {
        go_lock = false; // Tắt chữ GO! sau 0.5s
        needsDisplayUpdate = true;
    }

    // LOGIC VẼ MÀN HÌNH ĐỘNG (Dành cho trạng thái rảnh rỗi và chờ)
    static int last_shown_sec = -1;
    static uint32_t last_idle_draw = 0;

    //Biến Tracking để lọc nhiễu hiển thị
    static float last_disp_angle = -999.0;
    static int last_disp_dist = -1;
    static bool last_disp_lost = false;
    static bool is_full_redraw_needed = true; // Cờ yêu cầu vẽ lại bộ khung tĩnh

    if (currentState == STATE_IDLE) { // Giao diện chờ
        last_shown_sec = -1;
        uint32_t idle_duration = current_time - state_start_time;

        // Check 100ms một lần, chỉ đẩy dữ liệu qua I2C nếu có sự thay đổi thật sự
        if (current_time - last_idle_draw >= 100) {
            SystemData snap = sysBuffer[read_index.load()];

            // Định nghĩa mức độ thay đổi để trigger redraw - Deadband hiển thị
            // Lệch > 2 độ hoặc > 15mm thì mới cập nhật số, tránh nhiễu li ti làm nháy số
            bool angle_changed = fabsf(snap.enemy_angle - last_disp_angle) >= 2.0; 
            bool dist_changed = abs(snap.dist[0] - last_disp_dist) >= 15;          
            bool status_changed = (snap.isTargetLost != last_disp_lost);

            bool needs_update = is_full_redraw_needed || status_changed || (!snap.isTargetLost && (angle_changed || dist_changed));

            if (needs_update) {
                if (idle_duration < 30000) {  // Dưới 30 giây -> Thức chờ lệnh
                    if (is_full_redraw_needed) {
                        display.clearDisplay(); // Chỉ clear 1 lần duy nhất khi mới vào State
                        display.setTextSize(2);
                        display.setTextColor(SSD1306_WHITE);
                        display.setCursor(0, 0);
                        display.println("READY...");
                        is_full_redraw_needed = false;
                    }

                    // Partial Update: Tẩy riêng khu vực chứa text thông số ở nửa dưới (Y từ 16 đến 32)
                    display.fillRect(0, 16, 128, 16, SSD1306_BLACK);

                    display.setTextSize(1);
                    display.setCursor(0, 16); 
                    if (snap.isTargetLost) {
                        display.println("Target: LOST");
                    } else {
                        display.printf("Target: %.1f deg\n", snap.enemy_angle);
                    }
                    display.printf("Dist: %d mm\n", snap.dist[0]);
                } else {  // Quá 30 giây -> Lim dim ngủ
                    if (is_full_redraw_needed) {
                        display.clearDisplay();
                        display.setTextSize(4);
                        display.setTextColor(SSD1306_WHITE);
                        display.setCursor(28, 0);
                        display.print("u_u");
                        is_full_redraw_needed = false;
                    }

                    // Partial Update: Tẩy riêng khu vực góc trái trên cùng chứa cụm "D:XXX"
                    display.fillRect(0, 0, 40, 10, SSD1306_BLACK);

                    display.setTextSize(1);
                    display.setCursor(0, 0);
                    if(snap.dist[0] < 2000) {
                        display.printf("D:%d", snap.dist[0]);
                    } else {
                        display.print("D:INF");
                    }
                }

                display.display(); // Chốt xả dữ liệu qua I2C

                // Lưu lại trạng thái để so sánh cho chu kỳ tiếp theo
                last_disp_angle = snap.enemy_angle;
                last_disp_dist = snap.dist[0];
                last_disp_lost = snap.isTargetLost;
            }
            last_idle_draw = current_time;
        }
    }
    else if (currentState == STATE_INIT_DELAY) {
        is_full_redraw_needed = true;
        // Vẽ đếm ngược 3 -> 2 -> 1
        uint32_t elapsed = current_time - state_start_time;
        if (elapsed < 3000) {
            int seconds_left = 3 - (elapsed / 1000);
            if (seconds_left != last_shown_sec && seconds_left > 0) {
                display.clearDisplay();
                display.setTextSize(4);
                display.setCursor(55, 0);
                display.print(seconds_left);
                display.display();
                last_shown_sec = seconds_left;
            }
        }
    } 
    
    else if (currentState == STATE_CALIBRATION) {
        if (is_full_redraw_needed) {
            display.clearDisplay();
            display.setTextSize(2);
            display.setTextColor(SSD1306_WHITE);
            display.setCursor(0, 0);
            display.println("CALIB MODE");
            display.setTextSize(1);
            display.setCursor(0, 20);
            display.println("Motors: LOCKED");
            display.display();
            is_full_redraw_needed = false;
        }
    }

    else {
        is_full_redraw_needed = true;
        // Logic vẽ mặt
        last_shown_sec = -1; // Đảm bảo reset cờ đếm ngược

        if (needsDisplayUpdate) {
            if (go_lock) {
                // Vừa đếm xong 321, in chữ GO! 
                // trong 500ms
                display.clearDisplay();
                display.setTextSize(4);
                display.setCursor(30, 0);
                display.print("GO!");
                display.display();
            } else {
                drawCurrentFace(); // In mặt
            }
            needsDisplayUpdate = false;
        }
    }
    // Gửi telemetry 25Hz (chỉ khi DEBUG_LEVEL >=1)
#if DEBUG_LEVEL >= 1
    static uint32_t last_telemetry_time = 0;
    if (current_time - last_telemetry_time >= 40) { // 40ms một lần
        last_telemetry_time = current_time;
        
        // Đọc snapshot an toàn từ Double-Buffer thông qua cơ chế Lock-Free có sẵn của cậu
        SystemData telemetrySnap = sysBuffer[read_index.load()];
        
        // Đẩy dữ liệu ra Serial sang máy tính
        send_telemetry(telemetrySnap, currentState);
    }
#endif
    // Debug serial mỗi 500ms
#if DEBUG_LEVEL >= 2
    static uint32_t last_debug_time = 0;
    if (current_time - last_debug_time >= 500) { 
        last_debug_time = current_time;
        SystemData snap = sysBuffer[read_index.load()];
        printDebugInfo(snap);
    }
#endif

    vTaskDelay(pdMS_TO_TICKS(50));
}