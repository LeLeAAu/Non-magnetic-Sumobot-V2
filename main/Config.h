#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>           
#include <SparkFunLSM6DS3.h>   
#include <Adafruit_SSD1306.h>    

// 1. PIN MAPPING 
#define I2C_SDA 26
#define I2C_SCL 25
#define OLED_SDA 23
#define OLED_SCL 5


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

extern TwoWire I2COLED; 
extern Adafruit_SSD1306 display;

// 2. CONSTANTS & THRESHOLDS
// --- Khoảng cách (mm) ---
const uint16_t CONF_ENY = 1000;       // min distance xác nhận có đối thủ (tạm gán 600mm)
const uint16_t WARN_DIST = 350;      // ngưỡng cảnh báo đối thủ ở cự ly nguy hiểm
const uint16_t STRIKE_DIST = 250;    // khoảng cách bung lực tấn công (dành riêng cho d0)

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
const uint32_t EDGE_TIMEOUT = 300;      // thời gian phải sạch vạch trắng trước khi thoát RECOVER
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
const float PITCH_TH = 8.5;         // Ngưỡng góc nghiêng (độ) để xác định xe bị hất/rơi
const float ACC_IMPACT_TH = 1.2;     // Ngưỡng gia tốc (G) để nhận diện va chạm mạnh
const float SENSOR_SIN[5] = {0.0, -0.7071, 0.7071, -1.0, 1.0}; // Lookup table
const float SENSOR_COS[5] = {1.0,  0.7071, 0.7071,  0.0, 0.0}; // Lookup table
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

const uint8_t MEDIAN_WINDOW = 3;

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

// --- GLOBAL VARIABLES (Khai báo để các file khác biết nó tồn tại) ---
const char* getStateName(RobotState state);
extern SystemData sysData;
extern SemaphoreHandle_t dataMutex;
extern volatile RobotState currentState;
extern volatile RobotState previousState;
extern volatile bool needsDisplayUpdate;
extern bool go_lock;
extern uint32_t go_start_time;
extern uint32_t state_start_time;
extern bool state_just_entered;
extern uint16_t dist_history[5][3];
extern uint8_t dist_idx[5];

// Cho Motor/Sensor
extern VL53L1X sensorsToF[5]; 
extern LSM6DS3 myIMU;
extern TaskHandle_t TaskSensorHandle;
extern TaskHandle_t TaskFSMHandle;

#endif
