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

    // 3. THUẬT TOÁN TRACTION CONTROL (VUỐT GA CHỐNG TRƯỢT TIRE SLIP)
    static int current_pwm_L = 0;
    static int current_pwm_R = 0;
    static uint32_t last_ramp_time = 0;
    
    uint32_t dt = current_time - last_ramp_time;
    last_ramp_time = current_time;
    if (dt > 50) dt = 5; // Lọc nhiễu thời gian nếu OS bị suspend
    
    // Tốc độ tăng ga: 3 đơn vị PWM cho mỗi 1ms (Mất ~85ms để đạt max 255). 
    // Thông số này đủ gắt để húc, nhưng đủ mềm để lốp bắt được ma sát tĩnh.
    int max_step = dt * 3; 

    // Hàm Lambda xử lý vuốt ga thông minh
    auto smartRamp = [](int target, int current, int step) -> int {
        // Phanh khẩn cấp hoặc bị FSM ép đảo chiều gắt -> Cắt ga ngay lập tức
        if (target == 0 || (target > 0 && current < 0) || (target < 0 && current > 0)) {
            return target; 
        }
        // Đang lấy đà tiến lên
        if (target > 0 && target > current) return min(target, current + step);
        // Đang lấy đà lùi lại
        if (target < 0 && target < current) return max(target, current - step);
        
        // Đang giảm tốc độ (nhưng chưa về 0) -> Chấp nhận xả ga ngay
        return target; 
    };

    // Áp dụng bộ lọc cho tín hiệu đã qua xử lý Deadtime
    leftSpeed = smartRamp(leftSpeed, current_pwm_L, max_step);
    rightSpeed = smartRamp(rightSpeed, current_pwm_R, max_step);

    // Cập nhật bộ nhớ cho vòng lặp sau
    current_pwm_L = leftSpeed;
    current_pwm_R = rightSpeed;

    // XẢ TÍN HIỆU RA CHÂN VẬT LÝ
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

    // STATIC FRICTION COMPENSATOR
    // Thời gian chích xung PWM cao để thắng sức ỳ ban đầu
    const uint32_t KICKSTART_MS = 15;
    // Ngưỡng PWM để xem xét kích hoạt (nếu tốc độ yêu cầu nằm trong khoảng lờ đờ)
    const int SLOW_TURN_THRESHOLD = 120; 
    // Xung lực mồi (thường để 150-200, mình set mặc định 180 cho mượt)
    const int KICK_PWM = 220;

    uint32_t elapsed_in_state = millis() - state_start_time;

    // Chỉ can thiệp nếu FSM đang ở trong giai đoạn 30ms đầu tiên của 1 State mới
    if (elapsed_in_state < KICKSTART_MS) {
        
        // XỬ LÝ MOTOR TRÁI
        // Nếu lệnh yêu cầu đang bắt chạy chậm, nhưng lại không phải là lệnh phanh cứng (0)
        if (abs(leftSpeed) > 0 && abs(leftSpeed) < SLOW_TURN_THRESHOLD) {
            // Giữ nguyên chiều (dấu), chỉ bơm áp lên KICK_PWM
            leftSpeed = (leftSpeed > 0) ? KICK_PWM : -KICK_PWM;
        }

        // XỬ LÝ MOTOR PHẢI
        if (abs(rightSpeed) > 0 && abs(rightSpeed) < SLOW_TURN_THRESHOLD) {
            rightSpeed = (rightSpeed > 0) ? KICK_PWM : -KICK_PWM;
        }
    }

    // Sau khi lọc xong, đẩy xuống tầng dưới
    setMotors(leftSpeed, rightSpeed);

    int fsm_last_pwm = (leftSpeed + rightSpeed) / 2;

    active_pwm.store(fsm_last_pwm);
}
