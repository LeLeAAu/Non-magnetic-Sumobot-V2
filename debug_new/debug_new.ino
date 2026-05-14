#include "Config.h"
#include "MotorControl.h"
#include "DisplayFace.h"
#include "SensorTask.h"
#include "FSMTask.h"

// ================= CÁC BIẾN GLOBAL =================
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
uint16_t dist_history[5][3]; 
uint8_t dist_idx[5] = {0, 0, 0, 0, 0};

VL53L1X sensorsToF[5];
LSM6DS3 myIMU(I2C_MODE, 0x6B); 

// ================= BIẾN GIẢ LẬP PHẦN CỨNG =================
uint16_t sim_dist[5] = {8190, 8190, 8190, 8190, 8190};
uint16_t sim_line[4] = {4095, 4095, 4095, 4095}; // Trắng: < 500, Đen: ~4095
uint16_t sim_belly = 4095; // Bị đè: < 3000
float sim_pitch = 0.0;
float sim_roll = 0.0;
bool sim_impact = false;

// Bảng Lookup tính góc từ config gốc
const float MOCK_SIN[5] = {0.0, -0.7071, 0.7071, -1.0, 1.0}; 
const float MOCK_COS[5] = {1.0,  0.7071, 0.7071,  0.0, 0.0}; 

void TaskMockSensorCode(void * pvParameters) {
    for(;;) {
        SystemData tempData;
        xSemaphoreTake(dataMutex, portMAX_DELAY);
        tempData = sysData;
        xSemaphoreGive(dataMutex);

        // 1. MÔ PHỎNG DỮ LIỆU ToF & GÓC ĐỘ (Vector Math)
        float sum_x = 0.0, sum_y = 0.0, sum_weights = 0.0;
        float d_closest = 8190.0;
        
        for(int i=0; i<5; i++) {
            tempData.dist[i] = sim_dist[i];
            if (sim_dist[i] < CONF_ENY) { // CONF_ENY = 600
                float d_val = (float)sim_dist[i];
                float weight = 1000000.0f / (d_val * d_val);
                sum_x += MOCK_SIN[i] * weight;
                sum_y += MOCK_COS[i] * weight;
                sum_weights += weight;
                if (sim_dist[i] < d_closest) d_closest = sim_dist[i];
            }
        }

        if (sum_weights > 0.0) {
            tempData.isTargetLost = false;
            tempData.enemy_angle = atan2(sum_x, sum_y) * 180.0 / M_PI;
        } else {
            tempData.isTargetLost = true;
            tempData.enemy_angle = 0.0;
        }

        // 2. MÔ PHỎNG TCRT & LINE
        tempData.line[0] = sim_line[0]; // FL
        tempData.line[1] = sim_line[1]; // FR
        tempData.line[2] = sim_line[2]; // BL
        tempData.line[3] = sim_line[3]; // BR
        
        tempData.pitch = sim_pitch;
        tempData.roll = sim_roll;
        
        bool pitchUp   = (tempData.pitch > PITCH_TH);
        bool pitchDown = (tempData.pitch < -PITCH_TH); 
        bool isTipping = pitchUp || pitchDown || (fabsf(tempData.roll) > PITCH_TH);

        bool edge_FL = (!pitchUp) && (tempData.line[0] <= TCRT_EDGE_TH);
        bool edge_FR = (!pitchUp) && (tempData.line[1] <= TCRT_EDGE_TH);
        bool edge_BL = (!pitchDown) && (tempData.line[2] <= TCRT_EDGE_TH);
        bool edge_BR = (!pitchDown) && (tempData.line[3] <= TCRT_EDGE_TH);

        tempData.edgeDetect = edge_FL || edge_FR || edge_BL || edge_BR;
        tempData.liftDetected = (sim_belly <= TCRT_LIFT_TH);
        
        bool enemy_at_rear = (tempData.dist[3] < WARN_DIST || tempData.dist[4] < WARN_DIST);
        tempData.liftedFront = pitchUp && (tempData.dist[0] < WARN_DIST) && (!tempData.liftDetected);
        tempData.liftedRear  = pitchDown && enemy_at_rear;
        tempData.fallOut = (!tempData.liftDetected) && isTipping;
        tempData.beingLifted = (!tempData.liftDetected) && pitchUp && (tempData.dist[0] < WARN_DIST);
        tempData.impactDetected = sim_impact;

        // Trả cờ va chạm về false sau 1 nhịp để giống xung gia tốc
        if (sim_impact) sim_impact = false; 

        xSemaphoreTake(dataMutex, portMAX_DELAY);
        sysData = tempData;
        xSemaphoreGive(dataMutex);

        if (tempData.edgeDetect || tempData.fallOut || tempData.beingLifted || tempData.impactDetected) {
            if (TaskFSMHandle != NULL) xTaskNotifyGive(TaskFSMHandle);
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void setup() {
    Serial.begin(115200);
    
    I2COLED.begin(OLED_SDA, OLED_SCL, 400000); 
    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println(F("OLED fail"));
    } else {
        showLoading(); 
        display.clearDisplay();
        display.display();
    }

    Serial.println("\n>>> MOCK MODE V2: FUSION SIMULATION READY <<<");

    dataMutex = xSemaphoreCreateMutex();

    // HACK: Ép chân TTP223 thành OUTPUT để ta có thể dùng code kích hoạt giả lập trạng thái chạm
    pinMode(PIN_TTP223, OUTPUT);
    digitalWrite(PIN_TTP223, LOW);

    ledcAttach(PIN_MOTOR_L_RPWM, 20000, 8);
    ledcAttach(PIN_MOTOR_L_LPWM, 20000, 8);
    ledcAttach(PIN_MOTOR_R_RPWM, 20000, 8);
    ledcAttach(PIN_MOTOR_R_LPWM, 20000, 8);

    xTaskCreatePinnedToCore(TaskMockSensorCode, "TaskMockSensor", 10000, NULL, 1, &TaskSensorHandle, 0);
    xTaskCreatePinnedToCore(TaskFSMCode, "TaskFSM", 10000, NULL, 2, &TaskFSMHandle, 1);
}

void loop() {
    uint32_t current_time = millis();

    if (go_lock && (current_time - go_start_time >= 500)) {
        go_lock = false;
        needsDisplayUpdate = true; // Kích hoạt vẽ lại khuôn mặt tĩnh của FSM
    }

    // ================= PARSER LỆNH SERIAL =================
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        cmd.toLowerCase();

        // 1. TTP223 (Chạm cảm ứng)
        if (cmd == "ttp 1") {
            digitalWrite(PIN_TTP223, HIGH);
            Serial.println("[TTP223] = CHẠM (1)");
        } 
        else if (cmd == "ttp 0") {
            digitalWrite(PIN_TTP223, LOW);
            Serial.println("[TTP223] = NHẢ (0)");
        }
        // 2. NHẬP MẢNG KHOẢNG CÁCH: d 0 -45 45 -90 90
        else if (cmd.startsWith("d ")) {
            int d0, d1, d2, d3, d4;
            // Dùng sscanf bóc tách 5 số nguyên cách nhau bằng dấu cách
            if (sscanf(cmd.c_str(), "d %d %d %d %d %d", &d0, &d1, &d2, &d3, &d4) == 5) {
                sim_dist[0] = d0; sim_dist[1] = d1; 
                sim_dist[2] = d2; sim_dist[3] = d3; sim_dist[4] = d4;
                Serial.printf("[ToF] Đã chốt mảng: D0:%d | D_L:%d | D_R:%d | D_SL:%d | D_SR:%d\n", d0, d1, d2, d3, d4);
            } else {
                Serial.println("Lỗi cú pháp. Vd chuẩn: d 150 8190 8190 8190 8190");
            }
        }
        // 3. TCRT5000 (Line)
        else if (cmd.startsWith("line ")) {
            String target = cmd.substring(5);
            if (target == "fl") sim_line[0] = 100;
            else if (target == "fr") sim_line[1] = 100;
            else if (target == "bl") sim_line[2] = 100;
            else if (target == "br") sim_line[3] = 100;
            else if (target == "clear") {
                sim_line[0] = 4095; sim_line[1] = 4095; sim_line[2] = 4095; sim_line[3] = 4095;
            }
            Serial.println("[TCRT Line] Cập nhật!");
        }
        // 4. BỤNG (Belly / Lifted)
        else if (cmd == "belly 1") sim_belly = 100;
        else if (cmd == "belly 0") sim_belly = 4095;
        // 5. IMU
        else if (cmd.startsWith("pitch ")) sim_pitch = cmd.substring(6).toFloat();
        else if (cmd.startsWith("roll ")) sim_roll = cmd.substring(5).toFloat();
        else if (cmd == "impact") sim_impact = true;
    }

    // ================= OLED HIỂN THỊ =================
    if (currentState == STATE_IDLE) {
        static uint32_t last_idle_draw = 0;
        if (current_time - last_idle_draw >= 200) {
            display.clearDisplay();
            display.setTextSize(2);
            display.setCursor(0, 0);
            display.print("READY");
            
            SystemData snap;
            if (dataMutex) { xSemaphoreTake(dataMutex, portMAX_DELAY); snap = sysData; xSemaphoreGive(dataMutex); }
            
            display.setTextSize(1);
            display.setCursor(70, 0);
            if (snap.isTargetLost) display.print("LOST");
            else display.printf("%.0f deg", snap.enemy_angle);

            display.setCursor(0, 20);
            display.printf("D0: %d mm", snap.dist[0]);
            display.display();
            last_idle_draw = current_time;
        }
    } 
    else if (currentState == STATE_INIT_DELAY) {
        uint32_t elapsed = current_time - state_start_time;
        if (elapsed < 3000) {
            int sec = 3 - (elapsed / 1000);
            static int last_sec = -1;
            if (sec != last_sec) {
                display.clearDisplay();
                display.setTextSize(4);
                display.setCursor(55, 0);
                display.print(sec);
                display.display();
                last_sec = sec;
            }
        }
    } 
    else {
        if (needsDisplayUpdate) {
            drawCurrentFace();
            needsDisplayUpdate = false;
        }
    }

    // ================= LOG CONSOLE =================
    static uint32_t last_debug_time = 0;
    if (current_time - last_debug_time >= 500) { 
        last_debug_time = current_time;
        
        SystemData snap;
        RobotState state_snap;
        
        if (dataMutex != NULL) {
            xSemaphoreTake(dataMutex, portMAX_DELAY);
            snap = sysData;
            state_snap = currentState;
            xSemaphoreGive(dataMutex);
        }

        Serial.println("------------------------------------------------");
        Serial.printf("[FSM] %s | PWM: %d\n", getStateName(state_snap), snap.current_PWM);
        
        Serial.print("[ToF] ");
        for(int i=0; i<5; i++) { Serial.print(snap.dist[i]); Serial.print(" "); }
        if (snap.isTargetLost) Serial.println("| LOST");
        else Serial.printf("| Góc: %.1f\n", snap.enemy_angle);

        Serial.printf("[Sensors] Bụng: %d | Pitch: %.1f | Roll: %.1f\n", sim_belly, snap.pitch, snap.roll);
        
        if(snap.edgeDetect) Serial.print("!EDGE! ");
        if(snap.liftedFront) Serial.print("!LIFT_F! ");
        if(snap.fallOut) Serial.print("!FALL! ");
        Serial.println();
    }

    vTaskDelay(pdMS_TO_TICKS(50));
}