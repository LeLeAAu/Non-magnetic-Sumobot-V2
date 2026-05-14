#include "Config.h"
#include "MotorControl.h"
#include "DisplayFace.h"
#include "SensorTask.h"
#include "FSMTask.h"

TwoWire I2COLED = TwoWire(1);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &I2COLED, OLED_RESET);

SystemData sysData;
SemaphoreHandle_t dataMutex = NULL;
volatile RobotState currentState = STATE_IDLE;
volatile RobotState previousState = STATE_IDLE;
volatile bool needsDisplayUpdate = false;
bool go_lock = false;
uint32_t go_start_time = 0;
uint32_t state_start_time = 0;
bool state_just_entered = false;

TaskHandle_t TaskSensorHandle = NULL;
TaskHandle_t TaskFSMHandle = NULL;

uint16_t dist_history[5][3]; // MEDIAN_WINDOW = 3
uint8_t dist_idx[5] = {0, 0, 0, 0, 0};


VL53L1X sensorsToF[5]; 
const uint8_t VLX_ADDRESSES[5] = {0x30, 0x31, 0x32, 0x33, 0x34};

LSM6DS3 myIMU(I2C_MODE, 0x6B); 

void setup() {
    Serial.begin(115200);
    I2COLED.begin(OLED_SDA, OLED_SCL, 100000); // Khởi tạo I2C1
    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println(F("OLED fail"));
    } else {
        showLoading();
        display.clearDisplay();
        display.display();
    }
    Wire.begin(I2C_SDA, I2C_SCL); // <--- ÉP I2C CHẠY TRÊN CHÂN 26 VÀ 25
    Wire.setClock(400000);
    dataMutex = xSemaphoreCreateMutex();

    bool hardwareError = false;

    // --- Cấu hình GPIO cơ bản ---
    pinMode(PIN_TCRT_FL, INPUT);
    pinMode(PIN_TCRT_FR, INPUT);
    pinMode(PIN_TCRT_BL, INPUT);
    pinMode(PIN_TCRT_BR, INPUT);
    pinMode(PIN_TCRT_DETECT, INPUT);
    pinMode(PIN_TTP223, INPUT_PULLDOWN);

    // --- Cấu hình PWM cho Motor Driver ---
    ledcAttach(PIN_MOTOR_L_RPWM, 20000, 8);
    ledcAttach(PIN_MOTOR_L_LPWM, 20000, 8);
    ledcAttach(PIN_MOTOR_R_RPWM, 20000, 8);
    ledcAttach(PIN_MOTOR_R_LPWM, 20000, 8);

    // --- Khởi tạo IMU ---
    if (myIMU.begin() != 0) {
        Serial.println("IMU Error!");
        hardwareError = true;
    }

    // --- Quy trình gán địa chỉ động cho 5x VL53L1X ---
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
            Serial.print("Lỗi khởi tạo VL53L1X số "); Serial.println(i);
            hardwareError = true;
        } else {
            sensorsToF[i].setAddress(VLX_ADDRESSES[i]);
            
            // --- CẬP NHẬT MODE LONG ---
            sensorsToF[i].setDistanceMode(VL53L1X::Long); 
            // 33ms là Timing Budget tối thiểu cho mode Long để đảm bảo ổn định
            sensorsToF[i].setMeasurementTimingBudget(33000); 
            // Thời gian chờ giữa 2 lần lấy mẫu = Budget + 5ms margin
            sensorsToF[i].startContinuous(38);
        }
    }
    if (hardwareError) {
        display.clearDisplay();
        display.setTextSize(4);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(28, 0);
        display.print("X_X");
        display.display();
        
        Serial.println(">>> HARDWARE HALT: SENSOR FAILED! BOOT ABORTED. <<<");
        // Khóa cứng hệ thống ở đây, nháy LED hoặc chờ kỹ sư can thiệp
        while(true) {
            delay(100);
        }
    }

    Serial.println("Init phan cung xong!");

    // --- Khởi tạo FreeRTOS Tasks ---
    xTaskCreatePinnedToCore(TaskSensorCode, "TaskSensor", 10000, NULL, 1, &TaskSensorHandle, 0);
    xTaskCreatePinnedToCore(TaskFSMCode, "TaskFSM", 10000, NULL, 2, &TaskFSMHandle, 1);
}

void loop() {
    uint32_t current_time = millis();

    // 1. Quản lý cờ tắt chữ GO! sau 500ms
    if (go_lock && (current_time - go_start_time >= 500)) {
        go_lock = false;
        needsDisplayUpdate = true;
    }

    // 2. LOGIC VẼ MÀN HÌNH ĐỘNG (Dành cho trạng thái rảnh rỗi và chờ)
    static int last_shown_sec = -1;
    static uint32_t last_idle_draw = 0;

if (currentState == STATE_IDLE) {
        last_shown_sec = -1; // Reset biến đếm ngược
        uint32_t idle_duration = current_time - state_start_time;

        // Cập nhật màn hình IDLE mỗi 100ms (chống nhiễu I2C)
        if (current_time - last_idle_draw >= 100) {
            display.clearDisplay();
            display.setTextColor(SSD1306_WHITE);

            // 1. LẤY DATA (Dời lên đây để dùng chung cho cả 2 state)
            SystemData snap;
            if (dataMutex != NULL) {
                xSemaphoreTake(dataMutex, portMAX_DELAY);
                snap = sysData;
                xSemaphoreGive(dataMutex);
            }

            if (idle_duration < 30000) {  // Dưới 30 giây -> Thức chờ lệnh
                display.setTextSize(2);
                display.setCursor(0, 0);
                display.println("READY...");
                
                display.setTextSize(1);
                if (snap.isTargetLost) display.println("Target: LOST");
                else display.printf("Target: %.1f deg\n", snap.enemy_angle);

                // [OPTION 1] Thêm khoảng cách mắt giữa (D0) vào màn hình READY
                display.printf("Dist: %d mm\n", snap.dist[0]); 

            } else {  // Quá 30 giây -> Lim dim ngủ
                // 2. VẼ KHUÔN MẶT
                display.setTextSize(4);
                display.setCursor(28, 0);
                display.print("u_u");

                // 3. OVERLAY DỮ LIỆU VLX (Góc trên cùng bên trái)
                // Ép size nhỏ để tạo cảm giác HUD công nghệ
                display.setTextSize(1);
                display.setCursor(0, 0); 
                // Chỉ hiển thị mắt giữa d[0] cho gọn, màn 128x32 không nhét đủ 5 mắt đâu!
                if(snap.dist[0] < 2000) {
                    display.printf("D:%d", snap.dist[0]);
                } else {
                    display.print("D:INF");
                }
            }
            
            display.display();
            last_idle_draw = current_time;
        }
    }
    else if (currentState == STATE_INIT_DELAY) {
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
    else {
        // 3. LOGIC VẼ KHUÔN MẶT TĨNH (Khi bắt đầu đánh nhau)
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
                // In các khuôn mặt O.O, -_-, >_<
                drawCurrentFace();
            }
            needsDisplayUpdate = false;
        }
    }

    // --- 4. BẢNG ĐIỀU KHIỂN DEBUG (In ra Serial mỗi 500ms) ---
    static uint32_t last_debug_time = 0;
    if (current_time - last_debug_time >= 500) { 
        last_debug_time = current_time;
        
        SystemData snap;
        RobotState state_snap;
        uint32_t current_state_time = 0;
        
        if (dataMutex != NULL) {
            xSemaphoreTake(dataMutex, portMAX_DELAY);
            snap = sysData;
            state_snap = currentState;
            current_state_time = millis() - state_start_time;
            xSemaphoreGive(dataMutex);
        }

        Serial.println("================================================================");
        
        // Dòng 1: FSM & Thời gian
        Serial.print("[FSM] STATE: ");
        Serial.print(getStateName(state_snap));
        Serial.print(" | TimeInState: ");
        Serial.print(current_state_time);
        Serial.print(" ms | PWM Output: ");
        Serial.println(snap.current_PWM);

        // Dòng 2: Mắt thần (ToF) & Động học (Kinematics)
        Serial.print("[ToF] Dist: ");
        for(int i=0; i<5; i++) { 
            Serial.print(snap.dist[i]); Serial.print(" "); 
        }
        Serial.print(" | Target: ");
        if (snap.isTargetLost) {
            Serial.println("LOST");
        } else {
            Serial.print(snap.enemy_angle, 1);
            Serial.print(" deg | v_e: ");
            Serial.print(snap.v_e, 1);
            Serial.println(" mm/s");
        }

        // Dòng 3: Dò line & Cảm biến gầm (TCRT)
        Serial.print("[TCRT] Line: ");
        for(int i=0; i<4; i++) { 
            Serial.print(snap.line[i]); Serial.print(" "); 
        }
        Serial.print("| Bụng: ");
        Serial.println(analogRead(PIN_TCRT_DETECT));

        // Dòng 4: IMU & Cờ cảnh báo (Flags)
        Serial.print("[IMU] P: ");
        Serial.print(snap.pitch, 1);
        Serial.print(" | R: ");
        Serial.print(snap.roll, 1);
        Serial.print(" | [FLAGS]: ");

        if(!snap.edgeDetect && !snap.fallOut && !snap.liftedFront && !snap.liftedRear && !snap.beingLifted && !snap.impactDetected && !snap.closingFast && !snap.sideDanger && !snap.flkPossible) {
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
        Serial.println("\n");
    }

    vTaskDelay(pdMS_TO_TICKS(50));
}