/*
To do list
*/
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

// Khởi tạo bus I2C số 1 (I2C1) của ESP32 dành riêng cho OLED
TwoWire I2COLED = TwoWire(1);

const int XSHUT_PINS[5] = {27, 14, 13, 16, 17}; // Giữa, Trái, Phải, Sườn Trái, Sườn Phải

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

// 2. CONSTANTS & THRESHOLDS
// --- Khoảng cách (mm) ---
const uint16_t CONF_ENY = 600;       // min distance xác nhận có đối thủ (tạm gán 600mm)
const uint16_t WARN_DIST = 350;      // ngưỡng cảnh báo đối thủ ở cự ly nguy hiểm
const uint16_t STRIKE_DIST = 100;    // khoảng cách bung lực tấn công (dành riêng cho d0)

// --- Thời gian (ms) ---
const uint32_t MIN_STT_TIME = 50;    // thời gian tối thiểu giữ 1 state
const uint32_t PUSH_MS = 500;        // thời gian bơm xung lực đẩy
const uint32_t HOLD_PUSH_MS = 500;   // thời gian duy trì lực đẩy sau xung đầu
const uint32_t TIMEOUT_MAX = 1400;   // timeout chờ tối đa
const uint32_t ATK_LOCK_TIME = 500;  // thời gian tối đa trong STATE_ATK_LOCK

// --- Guard / Safety (ms) ---
const uint32_t SIDE_DANGER_TIME = 80;   // thời gian tối thiểu d báo < WARN_DIST để chốt sideDanger
const uint32_t FLK_STABLE_TIME = 100;   // thời gian flkPossible phải giữ TRUE liên tục
const uint32_t IGNORE_ANTI_PUSH = 200;  // thời gian mù bỏ qua IMU sau khi ra đòn đẩy
const uint32_t EDGE_TIMEOUT = 150;      // thời gian phải sạch vạch trắng trước khi thoát RECOVER
const uint32_t MAX_RECOVER_TIME = 800;  // thời gian tối đa cho 1 phase RECOVER
const uint32_t FLK_DEBOUNCE_TIME = 30;  // Thời gian ân hạn (chống dội) khi mất điều kiện tạt sườn

// --- Hệ thống ---
const uint8_t RETRY_LIMIT = 7;          // giới hạn số lần thử đẩy trong 1s để chống rung lắc
const uint8_t MAX_LOCK_RETRIES = 2;     // số lần tối đa ngắm/đẩy hụt trước khi reset state
const uint16_t TCRT_EDGE_TH = 500;      // Dưới 500 là vạch trắng
const uint16_t TCRT_LIFT_TH = 3000;     // Dưới 3000 là có bụng địch đè lên (Bình thường chĩa lên trời là ~4095)

// --- Kinematics Calibration ---
const float V_MAX_60 = 500.0;           // Vận tốc tiến tại PWM 150 (mm/s)
const float OMEGA_60 = 180.0;           // Vận tốc góc tại PWM 150 (degree/s)
const float BOT_HALF_WIDTH = 80.0;      // Nửa chiều rộng bot (mm) - Tùy chỉnh theo cơ khí
const float R_SIDE_MARGIN = 50.0;       // Khoảng cách an toàn margin (mm)
const float R_SIDE = BOT_HALF_WIDTH + R_SIDE_MARGIN;
const float PITCH_TH = 15.0;         // Ngưỡng góc nghiêng (độ) để xác định xe bị hất/rơi
const float ACC_IMPACT_TH = 0.6;     // Ngưỡng gia tốc (G) để nhận diện va chạm mạnh
const float SENSOR_SIN[5] = {0.0, -0.7071, 0.7071, -1.0, 1.0};
const float SENSOR_COS[5] = {1.0,  0.7071, 0.7071,  0.0, 0.0};
const float V_EMA_ALPHA = 0.25;         // Hệ số lọc EMA cho vận tốc tiếp cận (0.0 -> 1.0)
const float V_DEADBAND_MM = 5.0;        // Deadband loại bỏ nhiễu rung li ti

// --- Motor PWM (0-255) ---
const int PWM_MAX = 255;
const int PWM_STRIKE_HOLD = 220;
const int PWM_HIGH = 200;
const int PWM_JIGGLE = 177; // Lực đánh võng
const int PWM_MED = 150;
const int PWM_LOW = 100;
const int PWM_TURN_MIN = 80; // Thắng sức ỳ Worm Gear
const int PWM_PIVOT = 50; 

// --- Góc & Cự ly Kinematics ---
const float ANGLE_TIGHT = 5.0;   // Góc chính diện tuyệt đối
const float ANGLE_WIDE = 15.0;   // Góc cho phép tạt/ủi
const float ANGLE_LOST = 20.0;   // Góc lệch coi như hụt đòn
const float ANGLE_FLANK = 25.0;  // Góc kích hoạt Flank
const float ANGLE_REAR = -120.0;
const float ANGLE_BIN_RES = 5.0; // Độ phân giải Histogram
const float KP_STEERING = 4.0; 
const float ANGLE_SLOPPY = 30.0; // Góc lệch tối đa (vẫn cho phép ủi) khi đã rúc sát gầm địch
const uint8_t FEINT_CHANCE = 25; // Tỷ lệ % tung đòn giả (Feint)

const uint16_t DIST_BLIND = 2000; // Ngưỡng mù ToF
const uint16_t DIST_CLOSE = 150;  // Vùng tử thần áp sát gầm

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

struct SystemData {
    // 1. Raw Data
    uint16_t dist[5] = {8190, 8190, 8190, 8190, 8190}; 
    uint16_t line[4] = {0, 0, 0, 0};
    
    // 2. IMU & Kinematics Variables
    float pitch = 0.0, roll = 0.0, yaw = 0.0;
    float accelX = 0.0, accelY = 0.0, accelZ = 0.0;
    float enemy_angle = 0.0;        
    float v_0 = 0.0;                
    float v_e = 0.0;                
    float t_robot = 0.0, t_enemy = 9999.0;   
    int current_PWM = 0;
    
    // 3. Boolean Flags (Chỉ chứa data do Sensor Core 0 tạo ra)
    bool closingFast = false;
    bool flkPossible = false;
    bool fallOut = false;
    bool liftDetected = false;
    bool impactDetected = false;
    bool sideDanger = false;
    bool edgeDetect = false;
    bool isTargetLost = true; // Ban đầu auto mất mục tiêu
    bool liftedFront = false;  
    bool liftedRear = false;   
    bool beingLifted = false;
};

// Mutex bảo vệ sysData, còn currentState để volatile cho loop() đọc an toàn
SystemData sysData; 
volatile RobotState currentState = STATE_IDLE;
TaskHandle_t TaskSensorHandle;
TaskHandle_t TaskFSMHandle;

bool go_lock = false;
uint32_t go_start_time = 0;
volatile bool needsDisplayUpdate = false;

// Khởi tạo Spinlock Mutex cho ESP32 để chống Data Race
SemaphoreHandle_t dataMutex;
// Các biến quản lý State
uint32_t state_start_time = 0;
bool state_just_entered = false;
RobotState previousState = STATE_IDLE;

// Hàm này dọn dẹp tàn dư của State cũ trước khi vào State mới
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

// THÔNG SỐ BỘ LỌC VÀ ĐẠO HÀM
const uint8_t MEDIAN_WINDOW = 3; 
// Khởi tạo mảng history với giá trị 8190 (Out of range của ToF) thay vì 0
uint16_t dist_history[5][MEDIAN_WINDOW] = {
    {8190, 8190, 8190},
    {8190, 8190, 8190},
    {8190, 8190, 8190},
    {8190, 8190, 8190},
    {8190, 8190, 8190}
};
uint8_t dist_idx[5] = {0, 0, 0, 0, 0};

uint32_t last_derivative_time = 0;
uint16_t prev_filtered_d0 = 0;

// Hàm nội suy vận tốc tiến !Chỉnh lại sau khi debug thực tế
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

// ĐIỀU KHIỂN ĐỘNG CƠ (BTS7960) VỚI NON-BLOCKING DEADTIME
void setMotors(int leftSpeed, int rightSpeed) {
    // Ràng buộc giới hạn an toàn
    leftSpeed = constrain(leftSpeed, -PWM_MAX, PWM_MAX);
    rightSpeed = constrain(rightSpeed, -PWM_MAX, PWM_MAX);

    // Các biến static để ghi nhớ trạng thái giữa các vòng lặp
    static int last_left_sign = 0;   // 1: Tiến, -1: Lùi, 0: Dừng
    static int last_right_sign = 0;
    static uint32_t left_deadtime_start = 0;
    static uint32_t right_deadtime_start = 0;
    static bool left_in_deadtime = false;
    static bool right_in_deadtime = false;

    const uint32_t DEADTIME_MS = 30; // 30ms delay theo yêu cầu
    uint32_t current_time = millis();

    // 1. Lấy dấu của lệnh yêu cầu hiện tại
    int req_left_sign = (leftSpeed > 0) ? 1 : ((leftSpeed < 0) ? -1 : 0);
    int req_right_sign = (rightSpeed > 0) ? 1 : ((rightSpeed < 0) ? -1 : 0);

    // --- XỬ LÝ MOTOR TRÁI ---
    if (left_in_deadtime) {
        // Nếu đang trong thời gian om deadtime
        if (current_time - left_deadtime_start >= DEADTIME_MS) {
            left_in_deadtime = false;       // Đã xả xong, gỡ cờ
            last_left_sign = req_left_sign; // Chốt hướng chạy mới
        } else {
            leftSpeed = 0; // Vẫn chưa đủ 30ms, ép PWM = 0
        }
    } else {
        // Kiểm tra xem có sự "lật lọng" đột ngột từ Tiến sang Lùi (hoặc ngược lại) không
        if (last_left_sign != 0 && req_left_sign != 0 && last_left_sign != req_left_sign) {
            left_in_deadtime = true;
            left_deadtime_start = current_time;
            leftSpeed = 0; // Cắt động cơ ngay lập tức
        } else {
            // Không đảo chiều gắt, cập nhật hướng hiện tại
            if (req_left_sign != 0) last_left_sign = req_left_sign;
        }
    }

    // --- XỬ LÝ MOTOR PHẢI ---
    if (right_in_deadtime) {
        if (current_time - right_deadtime_start >= DEADTIME_MS) {
            right_in_deadtime = false;
            last_right_sign = req_right_sign;
        } else {
            rightSpeed = 0;
        }
    } else {
        if (last_right_sign != 0 && req_right_sign != 0 && last_right_sign != req_right_sign) {
            right_in_deadtime = true;
            right_deadtime_start = current_time;
            rightSpeed = 0;
        } else {
            if (req_right_sign != 0) last_right_sign = req_right_sign;
        }
    }

    // --- XẢ TÍN HIỆU RA CHÂN VẬT LÝ ---
    // Lúc này leftSpeed và rightSpeed đã được bộ lọc phía trên xử lý an toàn
    
    // Xả tín hiệu ra Motor Trái
    if (leftSpeed > 0) {
        ledcWrite(PIN_MOTOR_L_RPWM, leftSpeed);
        ledcWrite(PIN_MOTOR_L_LPWM, 0);
    } else if (leftSpeed < 0) {
        ledcWrite(PIN_MOTOR_L_RPWM, 0);
        ledcWrite(PIN_MOTOR_L_LPWM, abs(leftSpeed));
    } else {
        ledcWrite(PIN_MOTOR_L_RPWM, 0);
        ledcWrite(PIN_MOTOR_L_LPWM, 0);
    }

    // Xả tín hiệu ra Motor Phải
    if (rightSpeed > 0) {
        ledcWrite(PIN_MOTOR_R_RPWM, rightSpeed);
        ledcWrite(PIN_MOTOR_R_LPWM, 0);
    } else if (rightSpeed < 0) {
        ledcWrite(PIN_MOTOR_R_RPWM, 0);
        ledcWrite(PIN_MOTOR_R_LPWM, abs(rightSpeed));
    } else {
        ledcWrite(PIN_MOTOR_R_RPWM, 0);
        ledcWrite(PIN_MOTOR_R_LPWM, 0);
    }
}

// Hàm hỗ trợ đếm số lần xuất hiện (mode) trong histogram (dùng cho INIT_DELAY)
// Lượng tử hóa góc float thành các Bucket nguyên (mỗi bucket 5 độ)
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


// TASK FSM (CORE 1)

// Thêm hàm Helper thay thế cho setMotors() bên trong FSM
void driveBot(int leftSpeed, int rightSpeed) {

    // --- BỘ LỌC BÙ MA SÁT TĨNH (STATIC FRICTION COMPENSATOR) ---
    // Thời gian "chích" xung PWM cao để thắng sức ỳ ban đầu
    const uint32_t KICKSTART_MS = 30;
    // Ngưỡng PWM để xem xét kích hoạt (nếu tốc độ yêu cầu nằm trong khoảng lờ đờ)
    const int SLOW_TURN_THRESHOLD = 120; 
    // Xung lực mồi (thường để 150-200, mình set mặc định 180 cho mượt)
    const int KICK_PWM = 180;

    uint32_t elapsed_in_state = millis() - state_start_time;

    // Chỉ can thiệp nếu FSM đang ở trong giai đoạn 30ms đầu tiên của 1 State mới
    if (elapsed_in_state < KICKSTART_MS) {
        
        // --- XỬ LÝ MOTOR TRÁI ---
        // Nếu lệnh yêu cầu đang bắt chạy chậm, nhưng lại không phải là lệnh phanh cứng (0)
        if (abs(leftSpeed) > 0 && abs(leftSpeed) < SLOW_TURN_THRESHOLD) {
            // Giữ nguyên chiều (dấu), chỉ bơm áp lên KICK_PWM
            leftSpeed = (leftSpeed > 0) ? KICK_PWM : -KICK_PWM;
        }

        // --- XỬ LÝ MOTOR PHẢI ---
        if (abs(rightSpeed) > 0 && abs(rightSpeed) < SLOW_TURN_THRESHOLD) {
            rightSpeed = (rightSpeed > 0) ? KICK_PWM : -KICK_PWM;
        }
    }

    // Sau khi lọc xong, đẩy xuống tầng dưới
    setMotors(leftSpeed, rightSpeed);

    int fsm_last_pwm = (leftSpeed + rightSpeed) / 2;

    // ĐỒNG BỘ NGAY LẬP TỨC VÀO sysData
    if (dataMutex != NULL) { // Đảm bảo Mutex đã được khởi tạo
        xSemaphoreTake(dataMutex, portMAX_DELAY);
        sysData.current_PWM = fsm_last_pwm; 
        xSemaphoreGive(dataMutex);
    }
}

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

// Vẽ mặt lên 0.96LED 128x32
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
    int cy = 12; // Hạ tâm xuống 1 tí để cân bằng với text

    int r[3] = {4, 8, 12}; // 3 quỹ đạo
    float speeds[3] = {0.25, -0.15, 0.1}; // Vận tốc & hướng quay độc lập (Parallax)

    // ==================================================
    // PHASE 1: GYROSCOPE ORBIT (Con quay hồi chuyển)
    // ==================================================
    for (int frame = 0; frame < 80; frame++) {
        display.clearDisplay();

        // 1. Vẽ 3 quỹ đạo tĩnh
        for (int i = 0; i < 3; i++) {
            display.drawCircle(cx, cy, r[i], SSD1306_WHITE);
        }

        // 2. Vẽ 3 vệ tinh quay với tốc độ trượt nhau
        for(int i = 0; i < 3; i++) {
            // Lệch pha ban đầu + Tốc độ riêng biệt từng vòng
            float angle = (frame * speeds[i]) + (i * PI / 2.0); 
            int px = cx + cos(angle) * r[i];
            int py = cy + sin(angle) * r[i];
            
            display.fillCircle(px, py, 1, SSD1306_WHITE); // Chấm nhỏ 1px trông sắc nét (sleek) hơn
        }

        // 3. Xử lý chữ Loading tự động căn giữa siêu mượt
        int dots = (frame / 10) % 4;
        char loadStr[15] = "Loading";
        for (int d = 0; d < dots; d++) {
            strcat(loadStr, "."); // Cấp phát thêm dấu chấm
        }
        
        int textWidth = strlen(loadStr) * 6; // Tính lại chiều rộng mỗi frame
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(64 - (textWidth / 2), 24); // Ép căn giữa tuyệt đối
        display.print(loadStr);

        display.display();
        delay(20);
    }

    // ==================================================
    // PHASE 2: GRAVITY COLLAPSE (Sụp đổ hố đen)
    // ==================================================
    for(int step = 0; step <= 12; step += 2) { 
        display.clearDisplay();
        
        // Các vòng tròn co rút về tâm với tốc độ khác nhau
        if (r[2] - step > 0) display.drawCircle(cx, cy, r[2] - step, SSD1306_WHITE);
        if (r[1] - step/1.5 > 0) display.drawCircle(cx, cy, r[1] - step/1.5, SSD1306_WHITE);
        if (r[0] - step/3 > 0) display.drawCircle(cx, cy, r[0] - step/3, SSD1306_WHITE);
        
        display.fillCircle(cx, cy, 2, SSD1306_WHITE); // Giữ Core ở giữa

        // Vẽ lại chữ tĩnh
        display.setTextSize(1);
        display.setCursor(64 - (10 * 6) / 2, 24);
        display.print("Loading...");
        display.display();
        
        delay(30); 
    }

    // ==================================================
    // PHASE 3: CORE SHOCKWAVE & FLASH
    // ==================================================
    // Sóng xung kích bung ra
    for (int fw = 2; fw <= 24; fw += 6) {
        display.clearDisplay();
        display.drawCircle(cx, cy, fw, SSD1306_WHITE);
        display.fillCircle(cx, cy, fw/3, SSD1306_WHITE); // Lõi phình to dần
        display.display();
        delay(15);
    }
    
    // Cyber Flash (Invert đảo màu toàn bộ điểm ảnh trên OLED)
    display.invertDisplay(true); 
    delay(80); // Lóa mắt trong 80ms
    display.invertDisplay(false); // Trả lại bình thường
    display.clearDisplay();
    display.display();
    delay(200); // Blackout nghỉ lấy đà

    // ==================================================
    // PHASE 4: TACTICAL HUD BOOT (System Ready)
    // ==================================================
    for(int blink = 0; blink < 3; blink++) { 
        display.clearDisplay();
        
        // Vẽ thanh Header trên cùng
        display.fillRect(0, 0, 128, 7, SSD1306_WHITE);
        display.setTextColor(SSD1306_BLACK); // Chữ đen trên nền trắng
        display.setCursor(2, 0);
        display.print("OS_BOOT");

        // Vẽ viền HUD
        display.drawRect(0, 8, 128, 24, SSD1306_WHITE);
        
        // Chữ System Ready
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(16, 16);
        display.print("[ SYSTEM READY ]");
        
        display.display();
        delay(300); // Sáng lâu, uy lực

        display.clearDisplay();
        
        // Khung HUD giữ nguyên, chỉ chữ chớp tắt
        display.fillRect(0, 0, 128, 7, SSD1306_WHITE);
        display.setTextColor(SSD1306_BLACK);
        display.setCursor(2, 0);
        display.print("OS_BOOT");
        display.drawRect(0, 8, 128, 24, SSD1306_WHITE);
        
        display.display();
        delay(80); // Tối nhanh
    }

    // Chốt khung hình, chuẩn bị vào trận
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(16, 16);
    display.print("[ SYSTEM READY ]");
    display.display();
    delay(1500); 
}

// 5. SETUP & LOOP
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
                else display.printf("Target: %.1f deg\n", snap.enemy_angle);

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
