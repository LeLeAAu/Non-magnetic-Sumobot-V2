#include "Config.h"
// Constants và Thresholds
// --- Khoảng cách (mm) ---
volatile uint16_t CONF_ENY = 1000;      // min distance xác nhận có đối thủ (tạm gán 1000mm)
volatile uint16_t WARN_DIST = 350;      // ngưỡng cảnh báo đối thủ ở cự ly nguy hiểm
volatile uint16_t STRIKE_DIST = 250;    // khoảng cách bung lực tấn công (dành riêng cho d0)

// --- Thời gian (ms) ---
const uint32_t MIN_STT_TIME = 50;    // thời gian tối thiểu giữ 1 state
const uint32_t PUSH_MS = 500;        // thời gian bơm xung lực đẩy
const uint32_t HOLD_PUSH_MS = 500;   // thời gian duy trì lực đẩy sau xung đầu
const uint32_t TIMEOUT_MAX = 1400;   // timeout chờ tối đa
volatile uint32_t ATK_LOCK_TIME = 500;  // thời gian tối đa trong STATE_ATK_LOCK

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
volatile float V_MAX_60 = 500.0;           // Vận tốc tiến tại PWM 150 (mm/s)
volatile float OMEGA_60 = 180.0;           // Vận tốc góc tại PWM 150 (degree/s)
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
volatile uint8_t PWM_MAX = 255; // Toàn lực
volatile uint8_t PWM_STRIKE_HOLD = 220; // Duy trì áp lực đẩy
volatile uint8_t PWM_HIGH = 200; // Tốc độ cao
const uint8_t PWM_JIGGLE = 177; // Lực đánh võng
volatile uint8_t PWM_MED = 150; // Tốc độ trung bình
volatile uint8_t PWM_LOW = 100; // Tốc độ thấp
const uint8_t PWM_TURN_MIN = 80; // Lực tối thiểu để thắng ma sát tĩnh của hộp số Wormgear, chưa test thực tế, tạm để 80
const uint8_t PWM_PIVOT = 50;// Lực xoay tại chỗ chậm 

// --- Góc & Cự ly Kinematics ---
const float ANGLE_TIGHT = 5.0;   // Sai số góc cho phép coi là chính diện
const float ANGLE_WIDE = 15.0;   // Góc lệch cho phép tạt/ủi
const float ANGLE_LOST = 20.0;   // Góc lệch coi như hụt đòn
const float ANGLE_FLANK = 25.0;  // Góc kích hoạt Flank
const float ANGLE_REAR = -120.0; // Góc nhận diện địch phía sau
const float ANGLE_BIN_RES = 5.0; // Độ phân giải Histogram
volatile float KP_STEERING = 4.0;  // Hệ số tỉ lệ điều hướng
const float ANGLE_SLOPPY = 30.0; // Góc lệch tối đa (vẫn cho phép ủi) khi đã rúc sát gầm địch
volatile uint8_t FEINT_CHANCE = 25; // Tỷ lệ % tung đòn giả (Feint)

const uint16_t DIST_BLIND = 2000; // Khoảng cách coi như mù
const uint16_t DIST_CLOSE = 150;  // Vùng tử thần áp sát gầm

const uint8_t MEDIAN_WINDOW = 3; // Lọc Median cho cảm biến khoảng cách