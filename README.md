# English
# Non-magnetic-Sumobot-V2

## I. Overview
An autonomous ESP32-based robot specifically designed for the UTC SumoBot competition. The project focuses on asymmetric mechanical design and multithreaded processing (FreeRTOS) to optimize high-speed tactical reaction performance.

* **Status:** In Development.
* **License:** MIT License.

<img width="3300" height="2550" alt="Render" src="https://github.com/user-attachments/assets/b44ff627-61dc-4581-b793-258c26662c8b" />

---

## II. Directory Structure
* `/main`: Main source code and firmware.
* `/debug`: Testing and sensor calibration code.
* `/Documents`: Research documents, opponent meta analysis, and circuit schematics.
* `/Images`: Project diagrams and images.

---

## III. UTC Competition Rules
* **Dimensions:** Maximum 15x15cm, unlimited height.
* **Weight:** Maximum 1.5kg.
* **Arena (Dohyo):** Circular arena with a 100cm diameter and a 5cm white border. The surface is non-metallic, and magnets are prohibited.
* **Operation:** Fully autonomous. A mandatory 3-second delay before movement must be programmed.
* **Match Format:** 3 rounds, 3 minutes per round. Victory is achieved by flipping or pushing the opponent out of the arena. If both robots fall simultaneously, the robot that touches the ground last wins.

---

## IV. Mechanical & Weight Strategy
The total estimated hardware weight is approximately **1256g**. Instead of maximizing ballast to reach the 1.5kg limit, the bot adopts a "Lightweight & Agile" strategy:

1. **High Mobility:** Reduced inertia allows rapid acceleration and fast flanking maneuvers, exploiting the sluggish movement characteristics commonly found in Tanker-type bots.

2. **Opponent Counterplay:** Most opponents tend to expose their sides and rear. Despite the lower weight, the use of self-locking Worm Gear motors prevents the bot from being pushed backward. This is combined with a Teflon-coated wedge that stays close to the arena floor to lift the opponent's center of mass.

---

## V. Hardware Architecture
* **ESP32 Dual-Core (240MHz):** Processing tasks separated using FreeRTOS.
* **ToF Sensor Array (5x VL53L1X):** Provides 360-degree coverage. Independent XSHUT pins are used for sequential I2C address assignment during startup.
* **Infrared Sensors (5x TCRT5000):** Four edge-detection sensors for white border detection, plus one upward-facing underbody sensor to confirm when the opponent has been lifted onto the wedge.
* **IMU (LSM6DS3):** 6-axis accelerometer and gyroscope sensor. Detects impacts (G-force) and activates the Anti-Lift mechanism.
* **Power System:** High-discharge LiPo 3S 1500mAh 45C battery, Mini560PRO compact buck converter dedicated to the microcontroller power supply, and BTS7960 high-power motor drivers.

---

## VI. Software Architecture & FSM
The system operates using a non-blocking multithreaded architecture:

* **Core 0 (TaskSensor):** Continuously scans the I2C bus. Applies a Median filter for distance measurements and an EMA filter to estimate enemy approach velocity ($v_e$).

* **Core 1 (TaskFSM):** Controls the Finite State Machine (FSM) and PWM signal generation. Data shared between both cores is synchronized safely using a Spinlock Mutex.

**FSM (19 FSM States):**
* **Defensive States:** Edge Avoid, Anti-Lift, Anti-Push, Side Guard.
* **Offensive States:** Target Locking, Strike, Flank, Lift, Feint, Stalemate Brake.

---

## VII. ESP32 Pinout
* **Edge & Underbody Sensors (TCRT5000):**
    * Underbody Sensor (DETECT) -> Pin 18 (GPIO39).
    * Rear Left -> Pin 19 (GPIO34). Rear Right -> Pin 20 (GPIO35).
    * Front Left -> Pin 21 (GPIO32). Front Right -> Pin 22 (GPIO33).

* **Main I2C Bus:**
    * SCL -> Pin 23 (GPIO25).
    * SDA -> Pin 24 (GPIO26).

* **XSHUT Control (VL53L1X):**
    * Center (GPIO27), Left (GPIO14), Right (GPIO13), Left Side (GPIO16), Right Side (GPIO17).

* **Motor Driver (BTS7960):**
    * Right LPWM (GPIO18), Right RPWM (GPIO19), Left LPWM (GPIO21), Left RPWM (GPIO22).

* **UI Interface & Notes:**
    * Touch Start Button (TTP223) -> Pin 5 (GPIO4).
    * I2C OLED -> SDA (GPIO23), SCL (GPIO5).
    * **IMPORTANT WARNING:** Never connect components to Pin 27 (GPIO12), as it may cause boot failure issues. Avoid using RX0 and TX0 pins.
      
# Vietnamese
# Non-magnetic-Sumobot-V2
## I. Tổng quan
Robot tự hành ESP32 thiết kế riêng cho giải SumoBot trường UTC. Dự án tập trung vào thiết kế cơ khí bất đối xứng và xử lý đa luồng (FreeRTOS) để tối ưu phản xạ chiến thuật tốc độ cao.

* **Trạng thái:** Đang phát triển.
* **Giấy phép:** MIT License.
<img width="3300" height="2550" alt="Render" src="https://github.com/user-attachments/assets/b44ff627-61dc-4581-b793-258c26662c8b" />
---

## II. Cấu trúc thư mục
* `/main`: Mã nguồn và Firmware chính.
* `/debug`: Code test và hiệu chuẩn cảm biến.
* `/Documents`: Tài liệu nghiên cứu, phân tích meta đối thủ, sơ đồ mạch.
* `/Images`: Sơ đồ và hình ảnh dự án.

---

## III. Luật thi đấu UTC
* **Kích thước:** Tối đa 15x15cm, không giới hạn chiều cao.
* **Trọng lượng:** Tối đa 1.5kg.
* **Sàn đấu (Dohyo):** Tròn, đường kính 100cm, viền trắng 5cm. Sàn không kim loại, cấm sử dụng nam châm.
* **Vận hành:** Tự động 100%. Bắt buộc lập trình chờ 3 giây trước khi di chuyển.
* **Thể thức:** 3 hiệp, 3 phút/hiệp. Thắng khi đẩy lật/rớt đối thủ. Chạm đất sau sẽ thắng nếu cùng rớt.

---

## IV. Chiến lược Cơ khí & Trọng lượng
Tổng trọng lượng linh kiện kỹ thuật khoảng **1256g**. Thay vì cố nhồi nhét tạ cho đủ mốc 1.5kg, bot áp dụng chiến thuật "Nhẹ & Linh hoạt":
1.  **Linh hoạt cao:** Giảm quán tính để dễ dàng bứt tốc và xoay góc tạt sườn, tận dụng nhược điểm lù đù của các bot hệ Tanker.
2.  **Khắc chế đối thủ:** Đa số đối thủ thường hở sườn ngang và lưng. Dù nhẹ cân, nhưng việc sử dụng động cơ Worm Gear tự khóa cơ khí giúp bot không bị đẩy lùi. Kết hợp với lưỡi ủi bọc Teflon ép sát mặt sàn để hất bổng trọng tâm đối phương.

---

## V. Kiến trúc Phần cứng
* **ESP32 Dual-Core (240MHz):** Tách biệt xử lý bằng FreeRTOS.
* **Mắt thần ToF (5x VL53L1X):** Phủ sóng 360 độ. Sử dụng các chân XSHUT độc lập để gán địa chỉ I2C tuần tự khi khởi động.
* **Hồng ngoại (5x TCRT5000):** 4 cảm biến góc dò vạch trắng, 1 cảm biến gầm hướng lên trên để xác nhận khi đối thủ đã bị nạy lên lưỡi ủi.
* **IMU (LSM6DS3):** Cảm biến gia tốc 6 trục. Phát hiện va chạm (G-force) và kích hoạt cơ chế chống nhấc bổng (Anti-Lift).
* **Năng lượng:** Nguồn xả cao từ Pin LiPo 3S 1500mAh 45C, mạch giảm áp Mini560PRO siêu nhỏ cấp nguồn riêng cho vi điều khiển, và driver công suất BTS7960.

---

## VI. Kiến trúc Phần mềm & FSM
Hệ thống chạy đa luồng không tắc nghẽn (Non-blocking):
* **Core 0 (TaskSensor):** Quét I2C liên tục. Áp dụng bộ lọc Median cho dữ liệu khoảng cách và bộ lọc EMA để tính vận tốc tiếp cận của địch ($v_e$).
* **Core 1 (TaskFSM):** Điều khiển Cỗ máy trạng thái hữu hạn (FSM) và băm xung PWM. Dữ liệu giữa hai Core được khóa an toàn bằng Spinlock Mutex.

**FSM (19 trạng thái FSM):**
* **Phòng thủ:** Né mép sàn (Edge Avoid), Chống nạy mũi (Anti-Lift), Chống đẩy mù (Anti-Push), Thủ sườn (Side Guard).
* **Tấn công:** Điều hướng khóa góc mục tiêu (Lock), Đâm trực diện (Strike), Tạt sườn (Flank), Bẩy văng (Lift), Nhử đòn (Feint), Giằng co tự hãm (Stalemate Brake).

---

## VII. Sơ đồ chân ESP32 (Pinout)
* **Cảm biến gầm & mép (TCRT5000):**
    * Mắt gầm (DETECT) -> Chân 18 (GPIO39).
    * Trái sau -> Chân 19 (GPIO34). Phải sau -> Chân 20 (GPIO35).
    * Trái trước -> Chân 21 (GPIO32). Phải trước -> Chân 22 (GPIO33).

* **Bus I2C chính:**
    * SCL -> Chân 23 (GPIO25).
    * SDA -> Chân 24 (GPIO26).

* **Điều khiển XSHUT (VL53L1X):**
    * Giữa (GPIO27), Trái (GPIO14), Phải (GPIO13), Sườn trái (GPIO16), Sườn phải (GPIO17).

* **Cầu H (BTS7960):**
    * LPWM Phải (GPIO18), RPWM Phải (GPIO19), LPWM Trái (GPIO21), RPWM Trái (GPIO22).

* **Giao diện UI & Lưu ý:**
    * Nút cảm ứng khởi động (TTP223) -> Chân 5 (GPIO4).
    * I2C OLED -> SDA (GPIO23), SCL (GPIO5).
    * **CẢNH BÁO QUAN TRỌNG:** Tuyệt đối không cắm linh kiện vào chân 27 (GPIO12) để tránh lỗi Boot, bỏ qua chân RX0, TX0.

Sơ đồ nối dây tổng thể và các đi-ốt bảo vệ mạch đều được cung cấp cụ thể dưới dạng file thiết kế đồ họa trong thư mục /Documents (Fritzing).
