#include "Config.h"
#include "MotorControl.h"
#include <Arduino.h>

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

    const uint32_t DEADTIME_MS = 12; // 30ms delay theo yêu cầu
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

void driveBot(int leftSpeed, int rightSpeed) {

    // --- BỘ LỌC BÙ MA SÁT TĨNH (STATIC FRICTION COMPENSATOR) ---
    // Thời gian "chích" xung PWM cao để thắng sức ỳ ban đầu
    const uint32_t KICKSTART_MS = 15;
    // Ngưỡng PWM để xem xét kích hoạt (nếu tốc độ yêu cầu nằm trong khoảng lờ đờ)
    const int SLOW_TURN_THRESHOLD = 120; 
    // Xung lực mồi (thường để 150-200, mình set mặc định 180 cho mượt)
    const int KICK_PWM = 220;

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
