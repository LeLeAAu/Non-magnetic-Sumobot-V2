#include <Wire.h>
#include <VL53L1X.h>           
#include <SparkFunLSM6DS3.h>   
#include <math.h>     
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>         

VL53L1X sensorsToF[5];
const uint8_t VLX_ADDRESSES[5] = {0x30, 0x31, 0x32, 0x33, 0x34};

LSM6DS3 myIMU(I2C_MODE, 0x6B);

// 1. PIN MAPPING 
#define I2C_SDA 26
#define I2C_SCL 25
#define OLED_SDA 23
#define OLED_SCL 5

TwoWire I2COLED = TwoWire(1);

const int XSHUT_PINS[5] = {27, 14, 13, 16, 17};

#define PIN_TCRT_DETECT 39
#define PIN_TCRT_BL 34
#define PIN_TCRT_BR 35
#define PIN_TCRT_FL 32
#define PIN_TCRT_FR 33

#define PIN_MOTOR_L_RPWM 22
#define PIN_MOTOR_L_LPWM 21
#define PIN_MOTOR_R_RPWM 19
#define PIN_MOTOR_R_LPWM 18

#define PIN_TTP223 4

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &I2COLED, OLED_RESET);

// 2. CONSTANTS & THRESHOLDS (GIỮ NGUYÊN TỪ BẢN GỐC)
const uint16_t CONF_ENY = 600;
const uint16_t WARN_DIST = 350;
const uint16_t STRIKE_DIST = 100;
const uint32_t MIN_STT_TIME = 50;
const uint32_t PUSH_MS = 500;
const uint32_t HOLD_PUSH_MS = 500;
const uint32_t TIMEOUT_MAX = 1400;
const uint32_t ATK_LOCK_TIME = 500; 
const uint32_t SIDE_DANGER_TIME = 80;
const uint32_t FLK_STABLE_TIME = 100;
const uint32_t IGNORE_ANTI_PUSH = 200;
const uint32_t EDGE_TIMEOUT = 150;
const uint32_t MAX_RECOVER_TIME = 800;
const uint32_t FLK_DEBOUNCE_TIME = 30;
const uint8_t RETRY_LIMIT = 7;
const uint8_t MAX_LOCK_RETRIES = 2;
const uint16_t TCRT_EDGE_TH = 500;
const uint16_t TCRT_LIFT_TH = 3000;
const float V_MAX_60 = 500.0;
const float OMEGA_60 = 180.0;
const float BOT_HALF_WIDTH = 80.0;
const float R_SIDE_MARGIN = 50.0;
const float R_SIDE = BOT_HALF_WIDTH + R_SIDE_MARGIN;
const float PITCH_TH = 15.0;
const float ACC_IMPACT_TH = 0.6;
const float SENSOR_SIN[5] = {0.0, -0.7071, 0.7071, -1.0, 1.0};
const float SENSOR_COS[5] = {1.0,  0.7071, 0.7071,  0.0, 0.0};
const float V_EMA_ALPHA = 0.25;
const float V_DEADBAND_MM = 5.0;
const int PWM_MAX = 255;
const int PWM_STRIKE_HOLD = 220;
const int PWM_HIGH = 200;
const int PWM_JIGGLE = 177;
const int PWM_MED = 150;
const int PWM_LOW = 100;
const int PWM_TURN_MIN = 80;
const int PWM_PIVOT = 50;
const float ANGLE_TIGHT = 5.0;
const float ANGLE_WIDE = 15.0;  
const float ANGLE_LOST = 20.0;
const float ANGLE_FLANK = 25.0;  
const float ANGLE_REAR = -120.0;
const float ANGLE_BIN_RES = 5.0; 
const float KP_STEERING = 4.0; 
const float ANGLE_SLOPPY = 30.0;
const uint8_t FEINT_CHANCE = 25;
const uint16_t DIST_BLIND = 2000; 
const uint16_t DIST_CLOSE = 150;

// 3. DATA STRUCTURES & FSM ENUMS
enum RobotState {
    STATE_IDLE, STATE_INIT_DELAY,
    STATE_ATK_STRIKE, STATE_ATK_FLANK_FRONT, STATE_ATK_FLANK_SIDE, 
    STATE_ATK_FLANK_REAR, STATE_ATK_LIFT, STATE_ATK_FEINT, 
    STATE_ATK_DELAY_RUSH, STATE_ATK_LOCK, STATE_ATK_STALEMATE_BRAKE,
    STATE_DEF_ANTI_PUSH, STATE_DEF_SIDE_GUARD, STATE_DEF_REAR_GUARD, 
    STATE_DEF_EDGE_AVOID, STATE_DEF_ANTI_LIFT, STATE_DEF_LAST_STAND,
    STATE_REC_RECOVER, STATE_SEARCH_ENEMY
};

// HÀM HỖ TRỢ DEBUG IN TÊN STATE
const char* getStateName(RobotState state) {
    switch(state) {
        case STATE_IDLE: return "IDLE";
        case STATE_INIT_DELAY: return "INIT_DELAY";
        case STATE_ATK_STRIKE: return "ATK_STRIKE";
        case STATE_ATK_FLANK_FRONT: return "FLANK_FRONT";
        case STATE_ATK_FLANK_SIDE: return "FLANK_SIDE";
        case STATE_ATK_FLANK_REAR: return "FLANK_REAR";
        case STATE_ATK_LIFT: return "ATK_LIFT";
        case STATE_ATK_FEINT: return "ATK_FEINT";
        case STATE_ATK_DELAY_RUSH: return "DELAY_RUSH";
        case STATE_ATK_LOCK: return "ATK_LOCK";
        case STATE_ATK_STALEMATE_BRAKE: return "STALEMATE";
        case STATE_DEF_ANTI_PUSH: return "ANTI_PUSH";
        case STATE_DEF_SIDE_GUARD: return "SIDE_GUARD";
        case STATE_DEF_REAR_GUARD: return "REAR_GUARD";
        case STATE_DEF_EDGE_AVOID: return "EDGE_AVOID";
        case STATE_DEF_ANTI_LIFT: return "ANTI_LIFT";
        case STATE_DEF_LAST_STAND: return "LAST_STAND";
        case STATE_REC_RECOVER: return "RECOVER";
        case STATE_SEARCH_ENEMY: return "SEARCH_ENEMY";
        default: return "UNKNOWN";
    }
}

struct SystemData {
    uint16_t dist[5] = {8190, 8190, 8190, 8190, 8190};
    uint16_t line[4] = {0, 0, 0, 0};
    float pitch = 0.0, roll = 0.0, yaw = 0.0;
    float accelX = 0.0, accelY = 0.0, accelZ = 0.0;
    float enemy_angle = 0.0;        
    float v_0 = 0.0;
    float v_e = 0.0;                
    float t_robot = 0.0, t_enemy = 9999.0;   
    int current_PWM = 0;
    bool closingFast = false;
    bool flkPossible = false;
    bool fallOut = false;
    bool liftDetected = false;
    bool impactDetected = false;
    bool sideDanger = false;
    bool edgeDetect = false;
    bool isTargetLost = true; 
    bool liftedFront = false;
    bool liftedRear = false;   
    bool beingLifted = false;
};

SystemData sysData;
volatile RobotState currentState = STATE_IDLE;
TaskHandle_t TaskSensorHandle;
TaskHandle_t TaskFSMHandle;

bool go_lock = false;
uint32_t go_start_time = 0;
volatile bool needsDisplayUpdate = false;
SemaphoreHandle_t dataMutex;

uint32_t state_start_time = 0;
bool state_just_entered = false;
RobotState previousState = STATE_IDLE;

void enterState(RobotState newState) {
    RobotState oldState = currentState;
    previousState = oldState;
    currentState = newState;
    state_start_time = millis();
    state_just_entered = true;

    if (oldState == STATE_INIT_DELAY) {
        go_lock = true;
        go_start_time = millis();
        needsDisplayUpdate = true; 
    } else {
        if (newState == STATE_DEF_LAST_STAND) {
            go_lock = false;
            needsDisplayUpdate = true; 
        } 
        else if (!go_lock) { 
            needsDisplayUpdate = true;
        }
    }
}

const uint8_t MEDIAN_WINDOW = 3;
uint16_t dist_history[5][MEDIAN_WINDOW] = {
    {8190, 8190, 8190}, {8190, 8190, 8190}, {8190, 8190, 8190}, {8190, 8190, 8190}, {8190, 8190, 8190}
};
uint8_t dist_idx[5] = {0, 0, 0, 0, 0};
uint32_t last_derivative_time = 0;
uint16_t prev_filtered_d0 = 0;

float getEstimatedVelocity(int pwm) {
    if (pwm < 30) return 0;
    return ((float)pwm / 150.0) * V_MAX_60;
}

uint16_t getMedian(uint16_t* history_array, uint8_t size) {
    uint16_t a = history_array[0], b = history_array[1], c = history_array[2];
    if (a > b) { uint16_t tmp = a; a = b; b = tmp; }
    if (b > c) { uint16_t tmp = b; b = c; c = tmp; }
    if (a > b) { uint16_t tmp = a; a = b; b = tmp; }
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

// BẢN DEBUG: Vô hiệu hóa xuất tín hiệu phần cứng ra BTS7960
void setMotors(int leftSpeed, int rightSpeed) {
    // leftSpeed = constrain(leftSpeed, -PWM_MAX, PWM_MAX);
    // rightSpeed = constrain(rightSpeed, -PWM_MAX, PWM_MAX);
    // // ... logic xả xung đã bị tắt ...
}

float getModeAngle(float* history, int size) {
    int maxCount = 0;
    float modeVal = 999;
    int buckets[50];
    for (int i = 0; i < size; i++) buckets[i] = (int)(history[i] / ANGLE_BIN_RES);

    for (int i = 0; i < size; i++) {
        if (history[i] == 999.0) continue;
        int count = 0;
        for (int j = 0; j < size; j++) {
            if (buckets[j] == buckets[i]) count++;
        }
        if (count > maxCount) {
            maxCount = count;
            modeVal = history[i];
        }
    }
    return modeVal;
}

void driveBot(int leftSpeed, int rightSpeed) {
    const uint32_t KICKSTART_MS = 30;
    const int SLOW_TURN_THRESHOLD = 120;
    const int KICK_PWM = 180;
    uint32_t elapsed_in_state = millis() - state_start_time;

    if (elapsed_in_state < KICKSTART_MS) {
        if (abs(leftSpeed) > 0 && abs(leftSpeed) < SLOW_TURN_THRESHOLD) leftSpeed = (leftSpeed > 0) ? KICK_PWM : -KICK_PWM;
        if (abs(rightSpeed) > 0 && abs(rightSpeed) < SLOW_TURN_THRESHOLD) rightSpeed = (rightSpeed > 0) ? KICK_PWM : -KICK_PWM;
    }

    // Dummy call, phần cứng đã bị ngắt ở hàm setMotors
    setMotors(leftSpeed, rightSpeed);
    int fsm_last_pwm = (leftSpeed + rightSpeed) / 2;

    if (dataMutex != NULL) { 
        xSemaphoreTake(dataMutex, portMAX_DELAY);
        sysData.current_PWM = fsm_last_pwm; 
        xSemaphoreGive(dataMutex);
    }
}

void TaskFSMCode(void * pvParameters) {
    const int HIST_SIZE = 50;
    static float angle_histogram[HIST_SIZE];

    for (int i = 0; i < HIST_SIZE; i++) angle_histogram[i] = 999.0;
    static int hist_idx = 0;
    static uint8_t stalemate_cycles = 0;

    for(;;) {
        uint32_t fsm_current_time = millis();
        SystemData localData;
        
        xSemaphoreTake(dataMutex, portMAX_DELAY);
        localData = sysData;          
        xSemaphoreGive(dataMutex);

        bool is_self_jerk_blind_time = (fsm_current_time - state_start_time < 250);
        if (currentState != STATE_IDLE && currentState != STATE_INIT_DELAY) {

            if (localData.fallOut) {
                if (currentState != STATE_DEF_LAST_STAND) enterState(STATE_DEF_LAST_STAND);
            }
            else if (localData.liftedFront || localData.liftedRear || localData.beingLifted) {
                if (currentState != STATE_DEF_ANTI_LIFT) enterState(STATE_DEF_ANTI_LIFT);
            }
            else if (localData.edgeDetect) {
                if (currentState != STATE_DEF_EDGE_AVOID && currentState != STATE_DEF_LAST_STAND) enterState(STATE_DEF_EDGE_AVOID);
            }
            else if (localData.impactDetected && !is_self_jerk_blind_time && 
                     currentState != STATE_ATK_STRIKE && currentState != STATE_ATK_LIFT &&
                     currentState != STATE_DEF_ANTI_PUSH && currentState != STATE_DEF_REAR_GUARD &&
                     currentState != STATE_DEF_SIDE_GUARD) {
                
                bool blind_hit = (localData.dist[0] > WARN_DIST && localData.dist[1] > WARN_DIST && localData.dist[2] > WARN_DIST && localData.dist[3] > WARN_DIST && localData.dist[4] > WARN_DIST);
                bool side_hit = (localData.dist[3] < WARN_DIST || localData.dist[4] < WARN_DIST);
                
                if (blind_hit) enterState(STATE_DEF_REAR_GUARD);
                else if (side_hit && currentState == STATE_SEARCH_ENEMY) enterState(STATE_DEF_SIDE_GUARD);
                else enterState(STATE_DEF_ANTI_PUSH);
            }
        }

        switch (currentState) {
            case STATE_IDLE:
            {
                driveBot(0, 0);
                if (!localData.isTargetLost) {
                    angle_histogram[hist_idx] = localData.enemy_angle;
                    hist_idx = (hist_idx + 1) % HIST_SIZE;
                }
                if (digitalRead(PIN_TTP223) == HIGH) enterState(STATE_INIT_DELAY);
                break;
            }
            case STATE_INIT_DELAY: 
            {
                driveBot(0, 0);
                if (!localData.isTargetLost) {
                    angle_histogram[hist_idx] = localData.enemy_angle;
                    hist_idx = (hist_idx + 1) % HIST_SIZE;
                } else {
                    if (localData.dist[0] > DIST_BLIND && localData.dist[1] > DIST_BLIND && localData.dist[2] > DIST_BLIND && localData.dist[3] > DIST_BLIND && localData.dist[4] > DIST_BLIND) {
                        angle_histogram[hist_idx] = 180.0;
                        hist_idx = (hist_idx + 1) % HIST_SIZE;
                    }
                }

                if (currentState == STATE_IDLE) {
                    if (digitalRead(PIN_TTP223) == HIGH) enterState(STATE_INIT_DELAY);
                } 
                else if (currentState == STATE_INIT_DELAY) {
                    if (fsm_current_time - state_start_time >= 3000) {
                        float target_angle = getModeAngle(angle_histogram, HIST_SIZE);
                        if (localData.dist[0] < CONF_ENY) enterState(STATE_ATK_LOCK);
                        else {
                            enterState(STATE_SEARCH_ENEMY);
                            if (target_angle != 999.0) {
                                xSemaphoreTake(dataMutex, portMAX_DELAY);
                                sysData.enemy_angle = target_angle; 
                                xSemaphoreGive(dataMutex);
                            }
                        }
                    }
                }
                break;
            }
            case STATE_DEF_EDGE_AVOID:
            {
                static uint32_t clean_edge_time = 0;
                static int esc_l = -PWM_MAX, esc_r = -PWM_MAX; 

                if (state_just_entered) {
                    bool ignore_front = (localData.pitch > PITCH_TH);
                    bool ignore_rear  = (localData.pitch < -PITCH_TH);
                    bool edge_FL = (!ignore_front) && (localData.line[0] <= TCRT_EDGE_TH);
                    bool edge_FR = (!ignore_front) && (localData.line[1] <= TCRT_EDGE_TH);
                    bool edge_BL = (!ignore_rear)  && (localData.line[2] <= TCRT_EDGE_TH);
                    bool edge_BR = (!ignore_rear)  && (localData.line[3] <= TCRT_EDGE_TH);

                    if (edge_FL && edge_FR) { esc_l = -PWM_MAX; esc_r = -PWM_LOW; } 
                    else if (edge_BL && edge_BR) { esc_l = PWM_MAX; esc_r = PWM_LOW; } 
                    else if (edge_FL && edge_BR) { esc_l = -PWM_MAX; esc_r = PWM_MAX; } 
                    else if (edge_FR && edge_BL) { esc_l = PWM_MAX; esc_r = -PWM_MAX; } 
                    else if (edge_FL) { esc_l = -PWM_MAX; esc_r = -PWM_LOW; } 
                    else if (edge_FR) { esc_l = -PWM_LOW; esc_r = -PWM_MAX; } 
                    else if (edge_BL) { esc_l = PWM_MAX; esc_r = PWM_LOW; }   
                    else if (edge_BR) { esc_l = PWM_LOW; esc_r = PWM_MAX; }   
                    else { esc_l = -PWM_MAX; esc_r = -PWM_LOW; } 
                }
                driveBot(esc_l, esc_r);

                if (!localData.edgeDetect) {
                    if (clean_edge_time == 0) clean_edge_time = fsm_current_time;
                    if (fsm_current_time - clean_edge_time >= EDGE_TIMEOUT) {
                        clean_edge_time = 0;
                        if (localData.dist[0] < WARN_DIST) enterState(STATE_ATK_STRIKE);
                        else enterState(STATE_SEARCH_ENEMY);
                    }
                } else clean_edge_time = 0;
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
                    }
                } else stable_time = 0;
                break;
            }
            case STATE_DEF_ANTI_LIFT:
            {
                uint32_t elapsed_time = fsm_current_time - state_start_time;
                int escape_pwm = localData.liftedFront ? -PWM_MAX : PWM_MAX; 
                int jiggle_pwm = localData.liftedFront ? -PWM_JIGGLE : PWM_JIGGLE;
                
                if (elapsed_time < 200) driveBot(escape_pwm, escape_pwm);
                else if (elapsed_time < 800) {
                    int phase = ((elapsed_time - 200) / 100) % 2;
                    if (phase == 0) driveBot(escape_pwm, jiggle_pwm); 
                    else driveBot(jiggle_pwm, escape_pwm);            
                } 
                else driveBot(escape_pwm * 0.8, escape_pwm * 0.8);

                if (!localData.liftedFront && !localData.liftedRear && fabsf(localData.pitch) <= 5.0 && fabsf(localData.roll) <= 5.0) enterState(STATE_REC_RECOVER);
                else if (elapsed_time > 1200) enterState(STATE_DEF_LAST_STAND);
                break;
            }
            case STATE_DEF_ANTI_PUSH:
            {
                uint32_t elapsed_time = fsm_current_time - state_start_time;
                if (elapsed_time < 100) driveBot(0, 0); 
                else if (elapsed_time < 300) driveBot(-PWM_MAX, -PWM_MAX);
                else if (elapsed_time < 600) {
                    int phase = (elapsed_time / 50) % 2;
                    if (phase == 0) driveBot(-200, -PWM_PIVOT);
                    else driveBot(-PWM_PIVOT, -200);
                } 
                else enterState(STATE_REC_RECOVER);
                break;
            }
            case STATE_DEF_SIDE_GUARD:
            {
                uint32_t elapsed_time = fsm_current_time - state_start_time;
                static int escape_dir = 1;
                if (state_just_entered) escape_dir = (esp_random() & 1) ? 1 : -1;

                if (elapsed_time < 300) driveBot(PWM_MAX * escape_dir, PWM_MAX * escape_dir);
                else {
                    if (localData.dist[0] < WARN_DIST) enterState(STATE_ATK_LOCK);
                    else enterState(STATE_REC_RECOVER);
                }
                break;
            }
            case STATE_DEF_REAR_GUARD:
            {
                uint32_t elapsed_time = fsm_current_time - state_start_time;
                static int turn_dir = 1;
                if (state_just_entered) turn_dir = (localData.enemy_angle < 0) ? 1 : -1;

                if (elapsed_time < 400) {
                    if (turn_dir == 1) driveBot(PWM_MAX, 150);
                    else driveBot(150, PWM_MAX);
                } 
                else {
                    if (localData.flkPossible) enterState(STATE_ATK_FLANK_REAR);
                    else enterState(STATE_REC_RECOVER);
                }
                break;
            }
            case STATE_SEARCH_ENEMY:
            {
                uint32_t elapsed_time = fsm_current_time - state_start_time;
                static int search_dir = 1;
                if (state_just_entered) search_dir = (localData.enemy_angle < 0) ? -1 : 1;

                if (elapsed_time < 1000) driveBot(150 * search_dir, -150 * search_dir);
                else if (elapsed_time < 3000) driveBot(200 * search_dir, 50 * search_dir);
                else {
                    search_dir = -search_dir;
                    state_start_time = fsm_current_time; 
                }

                if (localData.dist[0] < CONF_ENY) {
                    if (localData.dist[0] <= WARN_DIST) enterState(STATE_ATK_STRIKE);
                    else enterState(STATE_ATK_LOCK);
                }
                else if (localData.dist[1] < CONF_ENY || localData.dist[2] < CONF_ENY || localData.dist[3] < CONF_ENY || localData.dist[4] < CONF_ENY) {
                    enterState(STATE_ATK_FLANK_SIDE);
                }
                break;
            }
            case STATE_REC_RECOVER:
            {
                static int random_turn_dir = 1;
                uint32_t elapsed_time = fsm_current_time - state_start_time;
                if (state_just_entered) random_turn_dir = (esp_random() & 1) ? 1 : -1;

                if (elapsed_time < 300) driveBot(-200, -200);
                else if (elapsed_time < MAX_RECOVER_TIME) driveBot(150 * random_turn_dir, -150 * random_turn_dir);
                
                if (elapsed_time >= 300) {
                    if (localData.dist[0] < WARN_DIST) enterState(STATE_ATK_LOCK);
                    else if (localData.dist[1] < WARN_DIST || localData.dist[2] < WARN_DIST || localData.dist[3] < WARN_DIST || localData.dist[4] < WARN_DIST) enterState(STATE_ATK_FLANK_SIDE);
                }

                if (elapsed_time >= MAX_RECOVER_TIME) enterState(STATE_SEARCH_ENEMY);
                break;
            }
            case STATE_ATK_LOCK:
            {
                static uint8_t lock_retries = 0;
                uint32_t elapsed_time = fsm_current_time - state_start_time;

                if (state_just_entered) stalemate_cycles = 0;
                if (localData.dist[0] > CONF_ENY && localData.dist[1] > CONF_ENY && localData.dist[2] > CONF_ENY && localData.dist[3] > CONF_ENY && localData.dist[4] > CONF_ENY) {
                    lock_retries++;
                    driveBot(0, 0); 
                    if (lock_retries >= MAX_LOCK_RETRIES) {
                        enterState(STATE_SEARCH_ENEMY);
                        lock_retries = 0;
                    } else state_start_time = fsm_current_time;
                    break;
                }

                float err_angle = localData.enemy_angle;
                int forward_pwm = 0;
                int turn_pwm = 0;

                if (fabsf(err_angle) <= ANGLE_TIGHT) driveBot(PWM_HIGH, PWM_HIGH);
                else {
                    turn_pwm = constrain(fabsf(err_angle) * KP_STEERING, PWM_TURN_MIN, PWM_HIGH);
                    if (fabsf(err_angle) < ANGLE_FLANK) forward_pwm = PWM_MED;
                    else forward_pwm = 0;

                    if (err_angle > 0) driveBot(forward_pwm + turn_pwm, forward_pwm - turn_pwm);
                    else driveBot(forward_pwm - turn_pwm, forward_pwm + turn_pwm);
                }

                bool is_ready_to_strike = false;
                if (localData.dist[0] < WARN_DIST && fabsf(err_angle) <= ANGLE_WIDE) is_ready_to_strike = true;
                else if (localData.dist[0] < DIST_CLOSE && fabsf(err_angle) <= ANGLE_SLOPPY) is_ready_to_strike = true;

                if (is_ready_to_strike) {
                    if (!localData.sideDanger) {
                        if (localData.closingFast) {
                            enterState(STATE_ATK_DELAY_RUSH);
                            lock_retries = 0;
                        } 
                        else {
                            if (localData.dist[0] > DIST_CLOSE && (esp_random() % 100 < FEINT_CHANCE)) {
                                enterState(STATE_ATK_FEINT);
                                lock_retries = 0;
                            } else {
                                enterState(STATE_ATK_STRIKE);
                                lock_retries = 0; 
                            }
                        }
                    }
                }
                
                if (elapsed_time > ATK_LOCK_TIME) {
                    if (localData.dist[0] < CONF_ENY) enterState(STATE_ATK_STRIKE);
                    else enterState(STATE_SEARCH_ENEMY);
                    lock_retries = 0;
                }
                break;
            }
            case STATE_ATK_STRIKE:
            {
                uint32_t elapsed_time = fsm_current_time - state_start_time;
                if (elapsed_time < PUSH_MS) driveBot(PWM_MAX, PWM_MAX);
                else if (elapsed_time < PUSH_MS + HOLD_PUSH_MS) driveBot(220, 220);
                else driveBot(200, 200);

                if (elapsed_time > IGNORE_ANTI_PUSH) {
                    if (localData.sideDanger == true && localData.impactDetected == true) {
                        enterState(STATE_DEF_ANTI_PUSH);
                        break; 
                    }
                }

                if (localData.liftDetected) {
                    enterState(STATE_ATK_LIFT);
                    break;
                }

                if (localData.dist[0] > WARN_DIST || fabsf(localData.enemy_angle) > 20.0) {
                    enterState(STATE_ATK_LOCK);
                    break;
                }

                if (elapsed_time > TIMEOUT_MAX) enterState(STATE_ATK_STALEMATE_BRAKE);
                break;
            } 
            case STATE_ATK_STALEMATE_BRAKE:
            {
                uint32_t elapsed_time = fsm_current_time - state_start_time;
                driveBot(0, 0);
                if (elapsed_time >= 250) {
                    if (localData.dist[0] <= WARN_DIST && fabsf(localData.enemy_angle) <= 15.0) {
                        stalemate_cycles++;
                        if (stalemate_cycles >= 5) {
                            stalemate_cycles = 0;
                            enterState(STATE_REC_RECOVER); 
                        } else enterState(STATE_ATK_STRIKE);
                    } else {
                        stalemate_cycles = 0;
                        enterState(STATE_ATK_LOCK);
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
                
                if (localData.impactDetected) enterState(STATE_ATK_STRIKE);
                else if (fabsf(err_angle) >= 25.0) enterState(STATE_ATK_FLANK_SIDE);
                else if (localData.dist[0] > CONF_ENY) enterState(STATE_SEARCH_ENEMY);
                break;
            }
            case STATE_ATK_FLANK_SIDE:
            {
                uint16_t min_left = min(localData.dist[1], localData.dist[3]);
                uint16_t min_right = min(localData.dist[2], localData.dist[4]);
                
                int thrust_pwm = PWM_MAX;
                int pivot_pwm = -PWM_PIVOT;

                if (localData.dist[0] < 200 || min_left < 150 || min_right < 150) {
                    pivot_pwm = -PWM_MAX;
                    thrust_pwm = PWM_MAX;
                }

                bool turn_left = false;
                if (min_left >= CONF_ENY && min_right >= CONF_ENY) turn_left = (localData.enemy_angle < 0);
                else if (fabsf((int)min_left - (int)min_right) <= 20) turn_left = (esp_random() & 1); 
                else if (min_left < min_right) turn_left = true;
                else turn_left = false;

                if (turn_left) driveBot(pivot_pwm, thrust_pwm);
                else driveBot(thrust_pwm, pivot_pwm);

                if (localData.impactDetected) enterState(STATE_ATK_STRIKE);
                else if (localData.dist[0] < CONF_ENY) {
                    if (localData.dist[0] <= WARN_DIST) enterState(STATE_ATK_STRIKE);
                    else enterState(STATE_ATK_LOCK);
                }
                else if (localData.dist[0] > CONF_ENY && localData.dist[1] > CONF_ENY && localData.dist[2] > CONF_ENY && localData.dist[3] > CONF_ENY && localData.dist[4] > CONF_ENY) {
                    if (fsm_current_time - state_start_time > 800) enterState(STATE_REC_RECOVER);
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

                if (localData.impactDetected) enterState(STATE_ATK_STRIKE);
                else if (localData.dist[0] < CONF_ENY) enterState(STATE_ATK_LOCK);
                else if (localData.dist[0] > CONF_ENY && localData.dist[1] > CONF_ENY && localData.dist[2] > CONF_ENY && localData.dist[3] > CONF_ENY && localData.dist[4] > CONF_ENY) {
                    if (fsm_current_time - state_start_time > 600) enterState(STATE_SEARCH_ENEMY);
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
                if (elapsed_time < 400) driveBot(180, 180);
                else {
                    if (localData.enemy_angle < 0) driveBot(-PWM_LOW, PWM_MAX);
                    else driveBot(PWM_MAX, -100);
                }

                if (elapsed_time >= 450) enterState(STATE_ATK_FLANK_SIDE);
                if (localData.impactDetected) enterState(STATE_ATK_STRIKE);
                if (localData.dist[0] > CONF_ENY && elapsed_time > 100) enterState(STATE_SEARCH_ENEMY);
                break;
            }
            case STATE_ATK_DELAY_RUSH:
            {
                uint32_t elapsed_time = fsm_current_time - state_start_time;
                driveBot(80, 80); 

                if (localData.dist[0] <= 150) enterState(STATE_ATK_STRIKE);
                else if (fabsf(localData.enemy_angle) > 20.0) enterState(STATE_ATK_LOCK);
                else if (localData.v_e <= 100.0) enterState(STATE_ATK_STRIKE);
                else if (elapsed_time > 600) enterState(STATE_ATK_STRIKE);
                break;
            }
            default:
                enterState(STATE_IDLE);
                driveBot(0, 0);
                break;
        }
        static RobotState last_processed_state = STATE_IDLE;
        if (last_processed_state == currentState) state_just_entered = false;
        last_processed_state = currentState;
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5));
    }
}

// 4. OLED RENDER
void drawCurrentFace() {
    display.clearDisplay();
    display.setTextSize(4);
    display.setTextColor(SSD1306_WHITE);
    
    if (currentState == STATE_ATK_LOCK) display.setCursor(28, 0), display.print("O.O");
    else if (currentState == STATE_SEARCH_ENEMY) display.setCursor(28, 0), display.print("-_-");
    else if (currentState == STATE_REC_RECOVER) display.setCursor(28, 0), display.print("@_@");
    else if (currentState == STATE_ATK_STALEMATE_BRAKE) display.setCursor(22, 0), display.print("=_=");
    else if (currentState == STATE_DEF_LAST_STAND) display.setCursor(22, 0), display.print("T_T");
    else if (currentState == STATE_ATK_FLANK_FRONT || currentState == STATE_ATK_FLANK_SIDE || currentState == STATE_ATK_FLANK_REAR) display.setCursor(22, 0), display.print("OwO");
    else if (currentState == STATE_ATK_STRIKE || currentState == STATE_ATK_LIFT || currentState == STATE_ATK_DELAY_RUSH || currentState == STATE_ATK_FEINT) display.setCursor(22, 0), display.print("MwM");
    else if (currentState >= STATE_DEF_ANTI_PUSH && currentState <= STATE_DEF_ANTI_LIFT) display.setCursor(28, 0), display.print(">_<");
    
    display.display();
}

void showLoading() {
    int cx = 64;
    int cy = 12;
    int r[3] = {4, 8, 12};
    float speeds[3] = {0.25, -0.15, 0.1};

    for (int frame = 0; frame < 80; frame++) {
        display.clearDisplay();
        for (int i = 0; i < 3; i++) display.drawCircle(cx, cy, r[i], SSD1306_WHITE);
        for(int i = 0; i < 3; i++) {
            float angle = (frame * speeds[i]) + (i * PI / 2.0);
            int px = cx + cos(angle) * r[i];
            int py = cy + sin(angle) * r[i];
            display.fillCircle(px, py, 1, SSD1306_WHITE);
        }

        int dots = (frame / 10) % 4;
        char loadStr[15] = "Loading";
        for (int d = 0; d < dots; d++) strcat(loadStr, ".");
        
        int textWidth = strlen(loadStr) * 6;
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(64 - (textWidth / 2), 24); 
        display.print(loadStr);
        display.display();
        delay(20);
    }

    for(int step = 0; step <= 12; step += 2) { 
        display.clearDisplay();
        if (r[2] - step > 0) display.drawCircle(cx, cy, r[2] - step, SSD1306_WHITE);
        if (r[1] - step/1.5 > 0) display.drawCircle(cx, cy, r[1] - step/1.5, SSD1306_WHITE);
        if (r[0] - step/3 > 0) display.drawCircle(cx, cy, r[0] - step/3, SSD1306_WHITE);
        
        display.fillCircle(cx, cy, 2, SSD1306_WHITE);
        display.setTextSize(1);
        display.setCursor(64 - (10 * 6) / 2, 24);
        display.print("Loading...");
        display.display();
        delay(30);
    }

    for (int fw = 2; fw <= 24; fw += 6) {
        display.clearDisplay();
        display.drawCircle(cx, cy, fw, SSD1306_WHITE);
        display.fillCircle(cx, cy, fw/3, SSD1306_WHITE); 
        display.display();
        delay(15);
    }
    
    display.invertDisplay(true);
    delay(80); 
    display.invertDisplay(false); 
    display.clearDisplay();
    display.display();
    delay(200); 

    for(int blink = 0; blink < 3; blink++) { 
        display.clearDisplay();
        display.fillRect(0, 0, 128, 7, SSD1306_WHITE);
        display.setTextColor(SSD1306_BLACK);
        display.setCursor(2, 0);
        display.print("OS_BOOT");
        display.drawRect(0, 8, 128, 24, SSD1306_WHITE);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(16, 16);
        display.print("[ SYSTEM READY ]");
        display.display();
        delay(300);

        display.clearDisplay();
        display.fillRect(0, 0, 128, 7, SSD1306_WHITE);
        display.setTextColor(SSD1306_BLACK);
        display.setCursor(2, 0);
        display.print("OS_BOOT");
        display.drawRect(0, 8, 128, 24, SSD1306_WHITE);
        display.display();
        delay(80);
    }

    display.setTextColor(SSD1306_WHITE);
    display.setCursor(16, 16);
    display.print("[ SYSTEM READY ]");
    display.display();
    delay(1500); 
}

// 5. SETUP & LOOP
void setup() {
    Serial.begin(115200);
    I2COLED.begin(OLED_SDA, OLED_SCL, 100000); 
    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println(F("OLED fail"));
    } else {
        showLoading();
        display.clearDisplay();
        display.display();
    }
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000);
    dataMutex = xSemaphoreCreateMutex();
    bool hardwareError = false;

    pinMode(PIN_TCRT_FL, INPUT);
    pinMode(PIN_TCRT_FR, INPUT);
    pinMode(PIN_TCRT_BL, INPUT);
    pinMode(PIN_TCRT_BR, INPUT);
    pinMode(PIN_TCRT_DETECT, INPUT);
    pinMode(PIN_TTP223, INPUT_PULLDOWN);

    // Xóa ledcAttach của Motors [cite: 468, 469]
    // ledcAttach(PIN_MOTOR_L_RPWM, 20000, 8); ...

    if (myIMU.begin() != 0) {
        Serial.println("IMU Error!");
        hardwareError = true;
    }

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
            Serial.print("Lỗi khởi tạo VL53L1X số ");
            Serial.println(i);
            hardwareError = true;
        } else {
            sensorsToF[i].setAddress(VLX_ADDRESSES[i]);
            sensorsToF[i].setDistanceMode(VL53L1X::Long);
            sensorsToF[i].setMeasurementTimingBudget(33000);
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
        while(true) delay(100);
    }

    Serial.println("Init phan cung xong!");
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

            if (idle_duration < 30000) {  // Dưới 30 giây -> Thức chờ lệnh
                display.setTextSize(2);
                display.setCursor(0, 0);
                display.println("READY...");
                
                display.setTextSize(1);
                // Copy data nhanh để lấy góc in ra màn hình
                SystemData snap;
                if (dataMutex != NULL) {
                    xSemaphoreTake(dataMutex, portMAX_DELAY);
                    snap = sysData;
                    xSemaphoreGive(dataMutex);
                }
                if (snap.isTargetLost) display.println("Target: LOST");
                else display.printf("Target: %.1f deg", snap.enemy_angle);

            } else {  // Quá 30 giây -> Lim dim ngủ
                display.setTextSize(4);
                display.setCursor(28, 0);
                display.print("u_u");
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
                // Vừa đếm xong 321, in chữ GO! trong 500ms
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

    // --- BẢNG ĐIỀU KHIỂN DEBUG (In ra Serial mỗi 500ms) ---
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
