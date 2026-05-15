/*
- Core 0: Chạy TaskSensor (Đọc ToF, IMU, TCRT, tính toán Kinematics)
- Core 1: Chạy TaskFSM (Ra quyết định trạng thái) và Loop (Cập nhật OLED/Debug)
- Giao tiếp liên tác vụ thông qua Mutex bảo vệ cấu trúc SystemData
*/

// Việc dùng ToF tạo khoảng delay đo khoảng cách khá lớn. Nếu sau này có một cuộc thi khác thì sẽ chuyển dùng cảm biến ánh sáng khác

// Todo list (dùng cho tất cả các file kể cả file main.ino này)
/*
*/

#include "Config.h"
#include "MotorControl.h"
#include "DisplayFace.h"
#include "SensorTask.h"
#include "FSMTask.h"

// Khởi tạo đối tượng hệ thống
TwoWire I2COLED = TwoWire(1);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &I2COLED, OLED_RESET);

// Quản lí dữ liệu và trạng thái toàn cục
SystemData sysData; // Kho lưu trữ dữ liệu cảm biến và biến động học
SemaphoreHandle_t dataMutex = NULL; // Có tác dụng ngăn chặn xung đột giữ liệu giữa 2 Core
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
uint16_t dist_history[5][3]; // MEDIAN_WINDOW = 3
uint8_t dist_idx[5] = {0, 0, 0, 0, 0};
VL53L1X sensorsToF[5]; 
const uint8_t VLX_ADDRESSES[5] = {0x30, 0x31, 0x32, 0x33, 0x34};
LSM6DS3 myIMU(I2C_MODE, 0x6B); 

void setup() {
    Serial.begin(115200);
    // Khởi tạo OLED sớm để hiện thị Boot
    I2COLED.begin(OLED_SDA, OLED_SCL, 100000); // Khởi tạo I2C1
    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println(F("OLED fail"));
    } else {
        showLoading(); // Animation khởi động
        display.clearDisplay();
        display.display();
    }

    // Cấu hình bus cảm biến chính
    Wire.begin(I2C_SDA, I2C_SCL); // <--- ÉP I2C CHẠY TRÊN CHÂN 26 VÀ 25
    Wire.setClock(400000); // Fast mode I2C
    dataMutex = xSemaphoreCreateMutex(); // Khởi tạo Mutex bảo mật dữ liệu

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

    // Khởi tạo IMU
    if (myIMU.begin() != 0) {
        Serial.println("IMU Error!");
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
            Serial.print("Lỗi khởi tạo VL53L1X số "); Serial.println(i);
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
        
        Serial.println(">>> HARDWARE HALT: SENSOR FAILED! BOOT ABORTED. <<<");
        // Khóa cứng hệ thống ở đây, nháy LED hoặc chờ kỹ sư can thiệp
        while(true) {
            delay(100);
        }
    }

    Serial.println("Init phan cung xong!");

    // Kích hoạt đa nhiệm, đẩy các Task vào các Core tương ứng
    xTaskCreatePinnedToCore(TaskSensorCode, "TaskSensor", 10000, NULL, 1, &TaskSensorHandle, 0);
    xTaskCreatePinnedToCore(TaskFSMCode, "TaskFSM", 10000, NULL, 2, &TaskFSMHandle, 1);
}

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

    if (currentState == STATE_IDLE) { // Giao diện chờ
        last_shown_sec = -1; // Reset biến đếm ngược
        uint32_t idle_duration = current_time - state_start_time;

        // Cập nhật màn hình IDLE mỗi 100ms (chống nhiễu I2C)
        if (current_time - last_idle_draw >= 100) {
            display.clearDisplay();
            display.setTextColor(SSD1306_WHITE);

            // Lấy data
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

                // Thêm khoảng cách mắt giữa (D0) vào màn hình READY
                display.printf("Dist: %d mm\n", snap.dist[0]); 

            } else {  // Quá 30 giây -> Lim dim ngủ
                // Vẽ mặt ngủ
                display.setTextSize(4);
                display.setCursor(28, 0);
                display.print("u_u");

                // Overlay dữ liệu VLX
                display.setTextSize(1);
                display.setCursor(0, 0); 
                // Chỉ hiển thị mắt giữa d[0] cho gọn
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

    // Debug serial mỗi 500ms
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

    vTaskDelay(pdMS_TO_TICKS(50)); // Giải phóng CPU cho các Task khác
}