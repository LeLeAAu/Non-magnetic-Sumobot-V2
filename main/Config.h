#ifndef CONFIG_H
#define CONFIG_H

// Thư viện hỗ trợ
#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>           
#include <SparkFunLSM6DS3.h>   
#include <Adafruit_SSD1306.h>   
#include <atomic> 
#include <freertos/semphr.h>

// DEBUG CONFIGURATION 
// 0: SILENT MODE - Dành cho thi đấu thực tế (Tắt toàn bộ Serial, tối ưu hiệu năng 100%)
// 1: EVENT MODE - Chỉ in các sự kiện quan trọng (Chuyển State, Lỗi phần cứng ngắt hệ thống)
// 2: VERBOSE MODE - In chi tiết tất cả thông số sensor, kinematics mỗi 500ms (Dùng khi test tại xưởng)
#define DEBUG_LEVEL 0

// Cấu hình Macro
#if DEBUG_LEVEL > 0
    #define DEBUG_PRINT(x) Serial.print(x)
    #define DEBUG_PRINTLN(x) Serial.println(x)
    #define DEBUG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
    #define DEBUG_PRINT(x)
    #define DEBUG_PRINTLN(x)
    #define DEBUG_PRINTF(...)
#endif


// Chân kết nối ESP32
// Giao tiếp I2C cho ToF và IMU
#define I2C_SDA 26
#define I2C_SCL 25
// Giao tiếp I2C riêng cho OLED nhằm tránh nhiễu bus cảm biến
#define OLED_SDA 23
#define OLED_SCL 5

// Chân điều khiển XSHUT để khởi tạo địa chỉ động cho 5 cảm biến ToF
const uint8_t XSHUT_PINS[5] = {27, 14, 13, 16, 17}; // Giữa, Trái, Phải, Sườn Trái, Sườn Phải

// Chân cảm biến dò line
#define PIN_TCRT_DETECT 39 // Phát hiện đối thủ
#define PIN_TCRT_BL 34 // Back-left
#define PIN_TCRT_BR 35 // Back-right
#define PIN_TCRT_FL 32 // Front-left
#define PIN_TCRT_FR 33 // Front-right

// Chân điều khiển Motor driver
#define PIN_MOTOR_L_RPWM 22
#define PIN_MOTOR_L_LPWM 21
#define PIN_MOTOR_R_RPWM 19
#define PIN_MOTOR_R_LPWM 18

// Chân cảm biến TTP223
#define PIN_TTP223 4

// Cấu hình màn hình 0.96LED 128x32
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

extern TwoWire I2COLED; 
extern Adafruit_SSD1306 display;



// Constants và Thresholds
// --- Khoảng cách (mm) ---
extern volatile uint16_t CONF_ENY;
extern volatile uint16_t WARN_DIST;
extern volatile uint16_t STRIKE_DIST;

// --- Thời gian (ms) ---
extern const uint32_t MIN_STT_TIME;
extern const uint32_t PUSH_MS;
extern const uint32_t HOLD_PUSH_MS;
extern const uint32_t TIMEOUT_MAX;
extern volatile uint32_t ATK_LOCK_TIME;

// --- Guard / Safety (ms) ---
extern const uint32_t SIDE_DANGER_TIME;
extern const uint32_t FLK_STABLE_TIME;
extern const uint32_t IGNORE_ANTI_PUSH;
extern const uint32_t EDGE_TIMEOUT;
extern const uint32_t MAX_RECOVER_TIME;
extern const uint32_t FLK_DEBOUNCE_TIME;

// --- Hệ thống ---
extern const uint8_t RETRY_LIMIT;
extern const uint8_t MAX_LOCK_RETRIES;
extern const uint16_t TCRT_EDGE_TH;
extern const uint16_t TCRT_LIFT_TH;

// --- Kinematics Calibration ---
extern volatile float V_MAX_60;
extern volatile float OMEGA_60;
extern const float BOT_HALF_WIDTH;
extern const float R_SIDE_MARGIN;
extern const float R_SIDE;
extern const float PITCH_TH;
extern const float ACC_IMPACT_TH;
extern const float SENSOR_SIN[5];
extern const float SENSOR_COS[5];
extern const float V_EMA_ALPHA;
extern const float V_DEADBAND_MM;

// --- Motor PWM (0-255) ---
extern volatile uint8_t PWM_MAX;
extern volatile uint8_t PWM_STRIKE_HOLD;
extern volatile uint8_t PWM_HIGH;
extern const uint8_t PWM_JIGGLE;
extern volatile uint8_t PWM_MED;
extern volatile uint8_t PWM_LOW;
extern const uint8_t PWM_TURN_MIN;
extern const uint8_t PWM_PIVOT;

// --- Góc & Cự ly Kinematics ---
extern const float ANGLE_TIGHT;
extern const float ANGLE_WIDE;
extern const float ANGLE_LOST;
extern const float ANGLE_FLANK;
extern const float ANGLE_REAR;
extern const float ANGLE_BIN_RES;
extern volatile float KP_STEERING;
extern const float ANGLE_SLOPPY;
extern volatile uint8_t FEINT_CHANCE;

extern const uint16_t DIST_BLIND;
extern const uint16_t DIST_CLOSE;

extern const uint8_t MEDIAN_WINDOW;





// Data Structures và FSM Enums
// Cấu trúc dữ liệu và trạng thái

// Định nghĩa các trạng thái FSM
enum RobotState {
    STATE_IDLE, STATE_INIT_DELAY,
    STATE_ATK_STRIKE, STATE_ATK_FLANK_FRONT, STATE_ATK_FLANK_SIDE, 
    STATE_ATK_FLANK_REAR, STATE_ATK_LIFT, STATE_ATK_FEINT, 
    STATE_ATK_DELAY_RUSH, STATE_ATK_LOCK, STATE_ATK_STALEMATE_BRAKE,
    STATE_DEF_ANTI_PUSH, STATE_DEF_SIDE_GUARD, STATE_DEF_REAR_GUARD, 
    STATE_DEF_EDGE_AVOID, STATE_DEF_ANTI_LIFT, STATE_DEF_LAST_STAND,
    STATE_REC_RECOVER, STATE_SEARCH_ENEMY
};

// Cấu trúc dữ liệu hệ thống
struct SystemData {
    // Raw Data
    uint16_t dist[5] = {8190, 8190, 8190, 8190, 8190}; // Khoảng cách từ 5 ToF
    uint16_t line[4] = {0, 0, 0, 0}; // Giá trị từ 4 cảm biến Line
    
    // IMU & Kinematics Variables
    float pitch = 0.0f, roll = 0.0f, yaw = 0.0f;
    float accelX = 0.0f, accelY = 0.0f, accelZ = 0.0f;
    float enemy_angle = 0.0f; // Góc của đối thủ so với trực chính của bot       
    float v_0 = 0.0f; // Vận tốc của bot
    float v_e = 0.0f; // Vận tốc tiếp cận của bot đối thủ
    float t_robot = 0.0f, t_enemy = 9999.0f; // Thời gian dự kiến để chạm mục tiêu
    uint16_t current_PWM = 0; // Giá trị PWM hiện tại đang xuất ra motor
    
    // Boolean Flags (Chỉ chứa data do Sensor Core 0 tạo ra)
    bool closingFast = false; // Đối thủ đang lao tới nhanh
    bool flkPossible = false; // Điều kiện tạt sườn
    bool fallOut = false; // Có dấu hiệu bị lật, rơi
    bool liftDetected = false; // Đã nâng được địch
    bool impactDetected = false; // Phát hiện có va chạm mạnh
    bool sideDanger = false; // Nguy hiểm từ phía sườn
    bool edgeDetect = false; // Phát hiện vạch trắng mép sân
    bool isTargetLost = true; // Đã mất dấu đối thủ
    bool liftedFront = false; // Bị nâng mũi
    bool liftedRear = false; // Bị nâng đuôi
    bool beingLifted = false; // Đang bị đối thủ nâng
    uint32_t timestamp = 0; // Đánh dấu thời gian dữ liệu xuất xưởng
    bool hardwareFailure = false; // Báo cáo cảm biến đã chết lâm sàng > 3 lần
};

// Khai báo biến toàn cục
const char* getStateName(RobotState state);

// Hệ thống Lock-Free Double-Buffer
extern SystemData sysBuffer[2];
extern std::atomic<uint8_t> read_index;
extern std::atomic<uint8_t> active_pwm;

// Trạng thái FSM & Kích hoạt OLED
extern volatile RobotState currentState;
extern volatile RobotState previousState;
extern volatile bool needsDisplayUpdate;
extern bool go_lock;
extern uint32_t go_start_time;
extern uint32_t state_start_time;
extern bool state_just_entered;

// Lịch sử cảm biến
extern uint16_t dist_history[5][3];
extern uint8_t dist_idx[5];

// Đối tượng Cảm biến & Task (Chạy đa luồng)
extern VL53L1X sensorsToF[5]; 
extern LSM6DS3 myIMU;
extern TaskHandle_t TaskSensorHandle;
extern TaskHandle_t TaskFSMHandle;

#endif
