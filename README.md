# English
# I. Overview
**Non-magnetic-Sumobot-V2** is an autonomous robotic system powered by the ESP32 microcontroller, specifically engineered for the competitive environment at UTC (University of Transport and Communications). 

The project focuses on the synergy between **Asymmetric Mechanical Design** and **Multi-threaded Data Processing**. Our core objective is to create an entity capable of instantaneous reflexes and tactical dominance through intelligent maneuvering on the **FreeRTOS** platform.

<img width="3300" height="2550" alt="Render" src="https://github.com/user-attachments/assets/d60f8a85-f728-4827-a75c-6e8db40fb1ce" />

## 1. Project Status
- **Status:** `Ongoing` (In active development). 
- *Note:* The project is currently pacing itself based on the author's budget constraints—engineering brilliance is infinite, but student funds are not.

## 2. License
- **LICENSE:** This project is released under the **MIT License**. You are free to copy, modify, distribute, and use the source code and hardware designs for both commercial and non-commercial purposes, provided that the original copyright notice and license terms are maintained.

---

# II. Repository Structure
To ensure maintainability and facilitate collaboration, the repository is organized into logical directories. This structure strictly separates executable code, research documentation, and debugging utilities.

* **/main**: Core project source code and firmware.
* **/debug**: Independent code snippets designed for unit testing and sensor calibration.
* **/Documents**: Comprehensive research papers, schematics, and technical specifications.
* **/Images**: Visual media, diagrams, and project photos.

---

# III. UTC Tournament Rules & Regulations
## 1. Physical Constraints
- **Dimensions:** Maximum **15cm x 15cm** (footprint). *No height limit.*
- **Weight:** Maximum **1.5kg**.
- **Dohyo (Arena):** A circular flat surface, **100cm** in diameter, with a **5cm** white border.
- **Magnet Ban:** Non-metallic surface. Tractive effort relies solely on the laws of static friction between tires and the floor: $$F = \mu \cdot N$$

## 2. Operational Requirements
- **100% Autonomous:** Any form of remote control or wireless communication is strictly prohibited.
- **Start Delay:** A mandatory **3-second delay** is required after activation before any movement occurs.

## 3. Match Format
- **Duration:** 3 rounds per match, maximum 3 minutes per round.
- **Victory Conditions:** Successfully pushing the opponent out of the ring or causing them to flip/lose mobility.
- **"Last Touch" Rule:** In the event of a simultaneous fall, the robot that touches the ground last is declared the winner.
- **Starting Positions:**
    * **Qualifiers:** Designated by the referee (Head-to-head, Back-to-back, or 45° angles).
    * **Finals:** Teams choose their own positions—introducing **Game Theory** elements to predict and counter the opponent's initial trajectory.

---

# IV. Mechanical Design & Weight Strategy
**Technical Weight Distribution Breakdown:**
* Worm Gear Motors (x2): **710g**
* PETG Chassis: **230g**
* 3S 1500mAh LiPo Battery: **150g**
* Wheels (x2): **60g**
* BTS7960 Motor Drivers (x2): **40g**
* Electronics (ESP32, Buck converter, Sensors, etc.): **~66g**
* **Total Technical Mass:** **~1256g**

While the regulations allow for up to 1.5kg, this project intentionally opts for a **"Light & Agile"** strategy (leaving ~244g for aesthetic "Deco" and shielding) based on two analytical points:

1.  **Inertia vs. Maneuverability:** Increasing static weight does not yield linear benefits. Excessive mass increases the **moment of inertia**, hindering rapid rotation and "flanking" maneuvers, while also draining the battery faster.
2.  **Scouting Intelligence:** Data suggests that most UTC competitors utilize rigid, "cookie-cutter" designs with predictable algorithms and side-profile blind spots. Therefore, extreme pushing force is deprioritized in favor of high-speed tactical positioning. The **Self-locking Worm Gear** system provides sufficient defensive "stiffness" even at 1.25kg.

---

# V. Competitor Ecosystem Deep Dive
For a detailed meta-analysis of common opponent designs and counter-strategies, please refer to the `/Documents` directory.

# VI. Hardware Architecture
The electronic backbone of the project is designed for high-speed data acquisition and high-torque power delivery.

### 1. ESP32 Dual-Core (Processing Power)
Leveraging the **Xtensa® Dual-Core 32-bit LX6** microprocessor (240MHz), the system implements true **Real-Time multitasking (RTOS)**:
* **Core 0:** Dedicated to high-speed I2C sensor bus communication and data sampling.
* **Core 1:** Reserved for kinematic calculations, FSM logic execution, and PWM motor control.

### 2. Time-of-Flight (ToF) Array
A 5-unit **VL53L1X** array provides 360-degree spatial awareness (Center, Left, Right, Left-Flank, Right-Flank).
* **I2C Address Re-assignment:** Since VL53L1X sensors share a default address (0x29), a sequential boot sequence is implemented. By toggling individual **XSHUT** pins via GPIO, the ESP32 assigns unique hardware addresses (0x30 - 0x34) to each sensor, enabling high-bandwidth parallel scanning with minimal wiring.

### 3. Line Detection & Underbelly Sensing
Five **TCRT5000** infrared reflective modules are deployed:
* **Four Corner Sensors:** Detect the white boundary (Dohyo edge) to trigger emergency avoidance.
* **"The Detect Eye" (Underbelly Sensor):** Mounted upward on the bulldozer blade. Its sole purpose is to detect the opponent's chassis once successfully wedged. This triggers the **Lift-and-Push** mode, overriding standard pursuit logic to finish the kill.

### 4. 6-Axis Inertial Measurement Unit (IMU)
The **LSM6DS3** (I2C 0x6B) acts as the robot's "vestibular system":
* **Impact Detection:** Detects sudden G-force spikes (>0.6G) to confirm physical contact even if optical sensors are blinded.
* **Anti-Lift Protocol:** Monitors **Pitch** angles. If the nose exceeds 15°, the system recognizes it is being lifted and triggers an emergency reverse maneuver.

### 5. Power Management
* **Source:** 3S LiPo (11.1V - 12.6V) 1500mAh 45C.
* **Regulators:** Dual **Mini560PRO** synchronous buck converters (dedicated 5V and 3.3V rails).
* **Motor Drivers:** High-current **BTS7960** H-Bridge modules.

---

# VII. Software Architecture & RTOS
The firmware is built on a non-blocking, multi-threaded paradigm.

### 1. Multi-Core Task Distribution
1.  **TaskSensorHandle (Core 0):** Samples 5x ToF, 5x TCRT, and IMU data. This ensures the sampling rate remains constant and is never jittered by motor control loops.
2.  **TaskFSMHandle (Core 1):** Consumes "cleaned" data to update the **Finite State Machine (FSM)** and output PWM signals.

### 2. Data Synchronization (Spinlock Mutex)
To prevent **Race Conditions** between the sampling core and the logic core, all system variables are protected via **Spinlock Mutex** (`dataMutex`).
> [!IMPORTANT]
> The logic core is "locked" during memory writes to ensure the robot never acts on partial or corrupted sensor packets.

### 3. Signal Processing & Filtering
* **Median Filter:** ToF distance data passes through a 3-window sorting network to eliminate "spikes" and environmental noise.
* **Exponential Moving Average (EMA):** Relative velocity ($v_e = \Delta d / \Delta t$) is smoothed using an $\alpha = 0.25$ coefficient to filter out mechanical vibrations.

### 4. Optimized Kinematics
* **Estimated Velocity:** Real-time speed ($v_0$) is interpolated based on PWM duty cycles and empirical constants ($V_{max} \approx 500 mm/s$).
* **Look-up Tables (LUT):** To save CPU cycles, trigonometric functions for sensor angles (SIN/COS) are pre-calculated. Target vectors are computed using simple matrix multiplications, reducing latency to nanoseconds.

---

# VIII. Finite State Machine (FSM) & Tactics
The brain operates on a 19-state FSM managed by `enterState()`, which handles state transitions, anti-stuck timers, and OLED telemetry updates.

### 1. Defensive States
* **STATE_DEF_EDGE_AVOID:** Instant diagonal reverse triggered when TCRT falls below `TCRT_EDGE_TH`.
* **STATE_DEF_ANTI_LIFT:** Pitch-based recovery to escape "Wedge Lords."
* **STATE_DEF_ANTI_PUSH:** If IMU detects backward acceleration while the robot is moving forward, the system identifies a "stealth pusher" (in a blind spot) and counters with max PWM.
* **STATE_DEF_SIDE_GUARD:** Flank ToF triggers a rapid pivot to face the threat with the primary blade.

### 2. Offensive States
* **STATE_ATK_STRIKE:** Full-throttle (PWM 255) head-on collision when the target is within striking distance.
* **STATE_ATK_LOCK:** Proportional Steering ($K_p$) pursuit. Adjusts differential wheel speeds based on the target's relative angle.
* **STATE_ATK_FLANK:** The "Blind Spot Exploit." Instead of a head-on turn, the robot arcs around the opponent's ultrasonic field to strike their vulnerable side.
* **STATE_ATK_LIFT:** Triggered by the Underbelly Sensor. Enters a high-torque "pulse" mode to drive the airborne opponent off the ring.
* **STATE_ATK_FEINT:** A 25% chance "juke" move—pulsing forward and stopping abruptly to bait the opponent's sensor reaction.
* **STATE_ATK_STALEMATE_BRAKE:** When $v=0$ during a push, the robot "jiggles" its wheels (PWM oscillation) to break the mechanical equilibrium and find a gap under the enemy's blade.

### 3. Search Strategy
* **STATE_SEARCH_ENEMY:** Default scanning mode using `PWM_TURN_MIN`. The power is precisely tuned to overcome the **Worm Gear** internal friction for a smooth, consistent rotation without jitter.

# IX. ESP32 Pinout Mapping

| Function | Component Name | ESP32 Pin (GPIO) | Notes & Hardware Installation |
| :--- | :--- | :--- | :--- |
| **Line & Underbelly Sensing** | Underbelly Eye (DETECT) | Pin 18 (GPIO39/SVN) | Detects if the opponent is successfully wedged. |
| | Back-Left (BL) | Pin 19 (GPIO34) | Edge detection - Rear left. |
| | Back-Right (BR) | Pin 20 (GPIO35) | Edge detection - Rear right. |
| | Front-Left (FL) | Pin 21 (GPIO32) | Edge detection - Front left. |
| | Front-Right (FR) | Pin 22 (GPIO33) | Edge detection - Front right. |
| **Core I2C Bus** | I2C SCL (5x ToF + IMU) | Pin 23 (GPIO25) | **MANDATORY:** Keep SDA/SCL wires twisted/close to minimize EMI. |
| | I2C SDA (5x ToF + IMU) | Pin 24 (GPIO26) | High-speed data synchronization. |
| **ToF Hardware Control** | XSHUT 0 (Center) | Pin 25 (GPIO27) | Hardware multiplexing for address re-assignment. |
| | XSHUT 1 (Left) | Pin 26 (GPIO14) | Hardware multiplexing for address re-assignment. |
| | XSHUT 2 (Right) | Pin 28 (GPIO13) | Hardware multiplexing for address re-assignment. |
| | XSHUT 3 (Left Flank) | Pin 6 (GPIO16/RX2) | **MANDATORY:** Route flank XSHUT wires away from motor leads. |
| | XSHUT 4 (Right Flank) | Pin 7 (GPIO17/TX2) | Route with XSHUT 3. |
| **Motor PWM Control** | Right LPWM | Pin 9 (GPIO18) | PWM Signal for Right BTS7960. |
| | Right RPWM | Pin 10 (GPIO19) | PWM Signal for Right BTS7960. |
| | Left LPWM | Pin 11 (GPIO21) | PWM Signal for Left BTS7960. |
| | Left RPWM | Pin 14 (GPIO22) | PWM Signal for Left BTS7960. |
| **User Interface (UI)** | Capacitive Touch (TTP223) | Pin 5 (GPIO4) | Triggers FSM Start & 3s Delay. |
| | I2C SDA (OLED 0.96) | Pin 15 (GPIO23) | Dedicated I2C bus to prevent sensor latency. |
| | I2C SCL (OLED 0.96) | Pin 8 (GPIO5) | Secondary I2C clock. |
| **Hardware Strapping** | LMS Module CS Pin | -> Pull-up to 3.3V | Sets I2C mode for IMU. |
| | LMS Module SA0 Pin | -> Pull-up to 3.3V | Sets hardware I2C address for IMU. |
| **FORBIDDEN PINS** | PIN 27 (GPIO12) | **IGNORE** | **STRONGLY PROHIBITED.** Can cause Boot failures if pulled High. |
| | Pins 12 & 13 (RX0, TX0) | **IGNORE** | Reserved for UART Flashing / Serial Debugging. |

> [!NOTE]
> Complete wiring schematics, including protection diode placements and decoupling capacitor maps, are available as high-resolution design files in the `/Documents` (Fritzing) directory.

# Vietnamese
# I. Tổng quan
Non-magnetic-Sumobot-V2 là hệ thống robot tự hành sử dụng vi điều khiển ESP32, được thiết kế chuyên biệt cho môi trường thi đấu tại UTC.
Dự án tập trung vào việc kết hợp giữa Thiết kế cơ khí bất đối xứng và Xử lý dữ liệu đa luồng. Mục tiêu là tạo ra một thực thể có khả năng phản xạ tức thì và chiếm ưu thế nhờ các chiến thuật cơ động thông minh trên nền tảng FreeRTOS.

<img width="3300" height="2550" alt="Render" src="https://github.com/user-attachments/assets/b44ff627-61dc-4581-b793-258c26662c8b" />


## 1. Thông tin trạng thái dự án
- Trạng thái: Ongoing (Đang trong quá trình phát triển). Tác giả chưa có đủ tiền để tiếp tục dự án.
## 2. Cấp phép bản quyền
- LICENSE: Dự án này được phát hành dưới giấy phép MIT License. Bạn được cấp quyền tự do sao chép, chỉnh sửa, phân phối, và sử dụng mã nguồn cũng như thiết kế phần cứng cho cả mục đích thương mại lẫn phi thương mại, với điều kiện duy nhất là phải giữ nguyên thông báo bản quyền và các điều khoản ghi trong tệp giấy phép gốc.

# II. Cấu trúc kho lưu trữ
Để đảm bảo tính khoa học, dễ dàng bảo trì và tạo điều kiện thuận lợi cho việc hợp tác phát triển, toàn bộ dự án được phân chia thành các thư mục logic sau đây. Việc tuân thủ nghiêm ngặt cấu trúc này giúp tách biệt rõ ràng giữa mã nguồn thực thi, tài liệu nghiên cứu và các công cụ gỡ lỗi.
* /main
- Thư mục cốt lõi của dự án.
* /debug
- Chứa các đoạn mã nguồn độc lập được thiết kế riêng cho mục đích kiểm thử và hiệu chuẩn.
* /Documents
- Lưu trữ toàn bộ các tài liệu nghiên cứu, sơ đồ và thông số kỹ thuật.
* /Images
- Dành riêng cho các tệp phương tiện trực quan.
# III. Quy định và thể thức giải đấu UTC
## 1. Giới hạn vật lí
- Kích thước: Tối đa 15cm x 15cm (mặt đáy). Không giới hạn chiều cao.
- Trọng lượng: Tối đa 1.5kg.
- Sàn đấu: Hình tròn đường kính 100cm, viền trắng 5cm.
- Cấm Nam châm: Sàn phi kim loại. Lực bám hoàn toàn dựa trên ma sát tĩnh giữa lốp và mặt sàn (μ⋅N)
## 2. Yêu cầu vận hành
- Tự động 100%: Nghiêm cấm điều khiển từ xa hoặc giao tiếp không dây.
- Trễ khởi động: Bắt buộc chờ 3 giây sau khi kích hoạt.
## 3. Thể thức
- Thời gian: 3 hiệp/trận, tối đa 3 phút/hiệp.
- Điều kiện thắng: Đẩy đối thủ ra khỏi sàn hoặc làm đối thủ mất khả năng di chuyển (lật).
- Quy tắc "Chạm đất sau": Nếu cả hai cùng rơi, robot chạm đất sau sẽ thắng.
- Vị trí xuất phát:
+ Vòng loại: Trọng tài chỉ định (đối đầu, đấu lưng, góc 45°).
+ Vòng chung kết: Đội tự chọn vị trí. Đây là bài toán Lý thuyết trò chơi (Game Theory) để dự đoán quỹ đạo tấn công của đối phương.

# IV. Thiết kế Cơ khí & Chiến lược Trọng lượng
Tổng hợp khối lượng của các linh kiện kỹ thuật cốt lõi:
- Động cơ Worm Gear (x2): 710g
- Khung PETG: 230g
- Pin 3S 1500mAh: 150g
- Bánh xe (x2): 60g
- Driver động cơ BTS7960 (x2): 40g
- Vi điều khiển ESP32, Mạch giảm áp, Cảm biến, Tụ điện, Điện trở: ~66g
Tổng khối lượng kỹ thuật cơ bản: ~1256g

Luật giải đấu cho phép trọng lượng tối đa lên tới 1.5kg (1500g). Theo tư duy cơ khí thông thường, đội phát triển nên gắn thêm khoảng 244g chì hoặc thép đặc vào gầm xe để tối đa hóa lực ma sát (tăng F). Tuy nhiên, lý do tại sao dự án này quyết định thay thế phần trọng lượng dư thừa bằng các chi tiết trang trí (Deco) thay vì tạ dựa trên hai luận điểm phân tích:
- Mức trọng lượng 1.5kg là quá thừa thãi đối với một giải đấu ở cấp độ này. Việc gia tăng trọng lượng tĩnh không mang lại lợi ích tuyến tính. Khối lượng quá nặng sẽ làm tăng mô-men quán tính quay, khiến robot xoay trở chật vật hơn khi cần thực hiện các pha "tạt sườn" (Flank) tốc độ cao, đồng thời rút cạn dòng điện xả của pin nhanh hơn.
- Dữ liệu trinh sát cho thấy đối thủ tại UTC phần lớn là sinh viên với các thiết kế rập khuôn, không phải là những "master" (bậc thầy) trong giới robot sumo chuyên nghiệp. Họ thường mắc lỗi trong thuật toán bám đuổi và thường bị mù ở hai bên sườn. Do đó, chúng ta không cần thiết phải đánh đổi sự linh hoạt để tối ưu lực đẩy đến mức cực đoan như ở các giải đấu đỉnh cao (nơi mọi robot đều hoàn hảo về phần mềm).

# V. Phân Tích Chuyên Sâu Hệ Sinh Thái Đối Thủ
- Vui lòng đọc trong thư mục Documents

# VI. Kiến Trúc Phần Cứng Điện Tử Của Dự Án
## 1. ESP32 Dual-Core
- Với vi xử lý lõi kép Xtensa 32-bit xung nhịp lên tới 240MHz, ESP32 mang lại khả năng xử lý đa nhiệm theo thời gian thực (RTOS). Một lõi sẽ chuyên tâm giao tiếp I2C với dải cảm biến ở tốc độ cao, trong khi lõi còn lại tập trung tính toán động học và đưa ra quyết định chiến thuật mà không hề bị cản trở bởi các vòng lặp chờ.
## 2. ToF Array
- Triển khai 5 VL53L1X Time-of-Flight, trải đều ở: Mắt giữa, Mắt trái, Mắt phải, Sườn trái, Sườn phải.
- Do các cảm biến VL53L1X đều chia sẻ chung một địa chỉ I2C mặc định trên phần cứng, việc giao tiếp đồng thời là bất khả thi theo chuẩn thông thường. Giải pháp kỹ thuật ở đây là nối chung toàn bộ chân SDA và SCL của cả 5 cảm biến vào 2 chân I2C duy nhất của ESP32. Để giải quyết xung đột địa chỉ, 5 chân GPIO độc lập từ ESP32 sẽ được kết nối tới chân XSHUT của từng cảm biến. Trong quá trình khởi động (Boot), ESP32 sẽ ngắt toàn bộ cảm biến, sau đó bật lần lượt từng chiếc một và sử dụng phần mềm để ghi đè (re-assign) cho chúng các địa chỉ I2C mới (từ 0x30 đến 0x34). Cơ chế này tiết kiệm tối đa tài nguyên chân cắm của vi điều khiển trong khi vẫn đảm bảo băng thông quét dữ liệu khổng lồ.
## 3. Line Detection & Chống nhấc bổng
- 5 module cảm biến phản xạ hồng ngoại TCRT5000 được triển khai.
- Bốn cảm biến được bố trí tại các góc (Trái trước, Phải trước, Trái sau, Phải sau) để đo cường độ phản xạ của vạch trắng.
- Cảm biến thứ 5: Mắt gầm (DETECT). Thay vì nhìn xuống đất, cảm biến này được thiết kế hướng lên phía trên mặt không gian của lưỡi ủi. Nhiệm vụ duy nhất của nó là phát hiện xem bề mặt bụng của đối thủ có đang đè lên lưỡi ủi của robot hay không. Nếu tín hiệu trả về báo hiệu sự che khuất, vi điều khiển sẽ nhận thức được rằng đòn ủi gầm đã thành công, từ đó kích hoạt chế độ nhồi ga liên tục để hất văng đối thủ.
## 4. Cảm biến quán tính 6 trục (IMU)
- Một module Gy-lsm6ds3 (chạy ở địa chỉ I2C 0x6B) được tích hợp trực tiếp vào hệ quy chiếu trọng tâm của robot. Nó có tác dụng đối phó với những tình huống mà cảm biến quang học bị mù. IMU liên tục cung cấp dữ liệu về gia tốc tuyến tính và vận tốc góc. Nếu gia tốc đột ngột vọt lên ngưỡng 0.6G, robot sẽ nhận thức được một pha va chạm vật lý (Impact) vừa xảy ra. Nếu góc Pitch (góc chúc mũi) vượt quá 15 độ, robot sẽ biết rằng nó đang bị đối thủ xúc lên khỏi mặt đất, từ đó kích hoạt khẩn cấp hàm lùi lại để tự cứu rỗi.
## 5. Quản lí năng lượng & Driver động cơ
- Năng lượng chính: pin LiPo Shangyi 1500mAh 45C 3S.
- Mạch giảm áp đồng bộ mini560PRO (Một ra 5V, một ra 3.3V)
- Driver động cơ: BTS7960
## 6. Misc
- Màn hình OLED 0.96
- Nút bấm cảm ứng điện dung TTP223
- Các tụ giảm nhiễu (470uF, 1000uF, 0,1uF etc)

# VII. Kiến Trúc Phần Mềm Và Hệ Điều Hành Thời Gian Thực
## 1. Phân chia tác vụ đa lõi.
1. TaskSensorHandle (Gắn trên Core 0): Trách nhiệm duy nhất của lõi này là liên tục đọc dữ liệu I2C từ 5 cảm biến ToF, đọc tín hiệu Analog/Digital từ 5 cảm biến TCRT, và truy xuất dữ liệu từ IMU. Việc gom nhóm này đảm bảo tốc độ lấy mẫu (Sampling rate) luôn đạt mức tối đa và không bao giờ bị nghẽn bởi các lệnh điều khiển động cơ.
2. TaskFSMHandle (Gắn trên Core 1): Lõi này lấy dữ liệu đã được làm sạch, tính toán động học và đưa ra các quyết định theo logic của Máy trạng thái hữu hạn (FSM), sau đó xuất tín hiệu băm xung PWM điều khiển tốc độ từng bánh xe.

## 2. Bảo vệ dữ liệu bằng Spinlock Mutex
- Với hai lõi chạy song song, thảm họa Xung đột dữ liệu (Data Race) rất dễ xảy ra. Hãy tưởng tượng Core 1 đang đọc tọa độ của đối thủ từ một biến hệ thống, nhưng Core 0 lại đột ngột ghi đè một giá trị mới vào ngay giữa lúc đọc. Kết quả là robot sẽ nhận được một tọa độ "rác", dẫn đến việc lao đầu vào tường.
- Để ngăn chặn điều này, dự án triển khai đối tượng SystemData bọc trong một Spinlock Mutex (dataMutex). Bất cứ khi nào Core 0 cần cập nhật cảm biến, nó sẽ "khóa" cửa bộ nhớ lại. Nếu Core 1 muốn đọc dữ liệu lúc đó, nó buộc phải chờ trong vài micro-giây cho đến khi Core 0 mở khóa. Cơ chế đồng bộ hóa này đảm bảo dữ liệu cảm nhận luôn nguyên vẹn và nhất quán tuyệt đối.

## 3. Thuật toán lọc nhiễu tín hiệu
- Lọc Trung vị (Median Filter): Mọi tín hiệu khoảng cách từ ToF đều được đưa qua một mảng lịch sử (window size = 3). Hàm getMedian sử dụng thuật toán mạng sắp xếp (sorting network) tốc độ cao để loại bỏ ngay lập tức các giá trị đột biến (Spikes). 
- Bộ lọc Trung bình động theo cấp số nhân (EMA Filter): Để dự đoán xem đối thủ đang lao tới hay lùi lại, robot tính toán Vận tốc tương đối ($v_e$) bằng cách lấy đạo hàm của khoảng cách theo thời gian ($\Delta d / \Delta t$). Tuy nhiên, đạo hàm số rất dễ bị dao động. Bộ lọc EMA với hệ số $\alpha = 0.25$ được áp dụng để làm mượt đường cong vận tốc, bỏ qua các rung động cơ khí siêu nhỏ.

## 4. Phân tích động học
- Hàm getEstimatedVelocity nội suy vận tốc tịnh tiến ($v_0$) hiện tại của robot dựa trên giá trị PWM đang cấp cho động cơ, quy chiếu theo một hằng số đã được hiệu chuẩn thực nghiệm (V_MAX_60 = 500 mm/s).
- Bên cạnh đó, việc xác định góc của mục tiêu (enemy_angle) không sử dụng các hàm lượng giác tốn tài nguyên của thư viện math.h. Thay vào đó, mã nguồn sử dụng các bảng hằng số Tra cứu trước (Look-up tables) như SENSOR_SIN và SENSOR_COS tương ứng với vị trí vật lý của 5 cảm biến ToF. Các phép nhân ma trận đơn giản này giúp ESP32 tính toán hướng mục tiêu với độ trễ tính bằng nano-giây.

# VIII. Máy trạng thái hữu hạn FSM và Logic chiến thuật
FSM gồm 19 trạng thái rời rạc với mỗi trạng thái có điều kiện kích hoạt, thực thi và điều kiện thoát riêng biệt, được điều phối qua hàm enterState(). Hàm này quản lý thời gian bắt đầu trạng thái để chống kẹt, đồng thời phất cờ (needsDisplayUpdate) để Core 0 cập nhật màn hình OLED mà không làm chậm nhịp độ chiến đấu.
## 1. Các trạng thái phòng thủ
- **STATE_DEF_EDGE_AVOID**: Khi bất kỳ cảm biến TCRT nào ghi nhận mức phản xạ giảm xuống dưới ngưỡng TCRT_EDGE_TH,  robot sẽ lập tức lùi lại theo góc chéo ngược với vị trí vạch, tự cứu mình khỏi rớt đài
- **STATE_DEF_ANTI_LIFT**: Dữ liệu IMU liên tục được giám sát. Nếu độ chúc mũi (Pitch) vượt quá 15 độ, robot hiểu rằng mặt gầm đang bị kênh lên bởi một đối thủ dạng Wedge_Lord. Robot lập tức trả ga giật lùi để thoát khỏi lưỡi ủi của địch.
- **STATE_DEF_ANTI_PUSH**: Tuyệt tác của việc áp dụng IMU. Nếu robot đang đứng yên hoặc cấp xung tiến tới, nhưng IMU lại đọc được một gia tốc lùi tuyến tính tàn bạo, hệ thống nhận ra có một kẻ tàng hình (do nằm trong vùng mù của ToF) đang đẩy mình từ góc hở. Trạng thái này lập tức bơm xung PWM tối đa để sinh lực kháng cự lại. Một biến thời gian IGNORE_ANTI_PUSH được cài đặt để làm mù IMU ngay sau khi chính robot của chúng ta thực hiện đòn tấn công, ngăn chặn việc FSM hiểu nhầm lực dội (recoil) do chính mình gây ra là một pha đẩy của địch.
- **STATE_DEF_SIDE_GUARD / REAR_GUARD**: Khi ToF sườn phát hiện vật thể tiếp cận với tốc độ cao, robot chủ động xoay thân để đưa lưỡi ủi mặt trước ra đỡ đòn, triệt tiêu ý đồ tạt sườn của địch.
## 2. Các trạng thái tấn công
- **STATE_ATK_STRIKE**: Khi đối thủ nằm chính diện (góc nhỏ) và tiến vào ngưỡng STRIKE_DIST, robot cấp PWM cực đại (PWM_MAX = 255) lao thẳng vào đụng độ vật lý.
- **STATE_ATK_LOCK**: Nếu đối thủ ở xa hơn nhưng vẫn trong tầm ngắm, robot chuyển sang chế độ "Khóa mục tiêu". Thuật toán lái xe theo Tỷ lệ (Proportional Steering) sử dụng hệ số KP_STEERING sẽ tính toán độ sai lệch góc (enemy_angle) và điểu chỉnh chênh lệch PWM giữa bánh trái và bánh phải, giúp robot bám đuổi đối thủ. Để chống kẹt khi đối thủ quá nhanh, trạng thái này bị giới hạn bởi ATK_LOCK_TIME.
- **STATE_ATK_FLANK**: Ứng dụng từ phân tích điểm mù của đối thủ. Nếu ToF sườn bắt được tín hiệu, robot không thèm quay mặt lại đấu đầu. Nó sẽ khóa một bánh, tăng tốc bánh kia để vẽ một đường vòng cung, lách qua mặt cảm biến siêu âm của địch và cắm thẳng lưỡi ủi vào sườn non mù lòa của chúng.
- **STATE_ATK_LIFT**: Khi cảm biến Mắt Gầm (PIN_TCRT_DETECT) xác nhận bụng đối thủ đã nằm ngọn trên lưỡi ủi (giá trị < 3000), robot chuyển sang duy trì ga nhồi (PWM_STRIKE_HOLD) để ủi bay một cỗ máy đã mất hoàn toàn lực ma sát tĩnh ra khỏi võ đài.
- **STATE_ATK_FEINT**: Khi phát hiện mục tiêu, có 25% xác suất (FEINT_CHANCE) FSM sẽ kích hoạt đòn giả. Robot sẽ nhích nhẹ tiến lên rồi khựng lại.
- **STATE_ATK_STALEMATE_BRAKE**: Khi hai Tanker húc nhau giằng co, động năng bằng 0. Trạng thái này sẽ băm xung PWM_JIGGLE để robot liên tục rùng mình, lách lưỡi ủi qua lại hòng phá vỡ thế cân bằng cơ học của bánh răng địch, tìm đường lách dưới gầm.
## 3. Các trạng thái tìm kiếm
- **STATE_SEARCH_ENEMY**: Hoạt động mặc định khi biến isTargetLost trả về True. Robot thực hiện xoay tròn với xung PWM_TURN_MIN. Việc áp dụng PWM ở mức 80 là một tinh chỉnh cơ khí tinh tế: Đây là mức năng lượng tối thiểu vừa đủ để thắng được lực cản ma sát tự thân bên trong hộp số Worm Gear, giúp robot xoay êm ái mà không bị giật

# IX. ESP32 Pinout Mapping
| Chức Năng | Tên Linh Kiện | Chân ESP32 (GPIO) | Ghi Chú & Lưu Ý Lắp Đặt Cứng |
| :--- | :--- | :--- | :--- |
| **Cảm Biến Line & Gầm** | Mắt gầm (DETECT) | Pin 18 (GPIO39/SVN) | Cảm biến ủi hất đối thủ. |
| | Mắt Trái sau (BL) | Pin 19 (GPIO34) | Chống rơi góc trái lùi. |
| | Mắt Phải sau (BR) | Pin 20 (GPIO35) | Chống rơi góc phải lùi. |
| | Mắt Trái trước (FL) | Pin 21 (GPIO32) | Chống rơi góc trái tiến. |
| | Mắt Phải trước (FR) | Pin 22 (GPIO33) | Chống rơi góc phải tiến. |
| **Bus Giao Tiếp I2C Lõi** | I2C SCL (5x VLX + LMS) | Pin 23 (GPIO25) | **BẮT BUỘC:** Gom 2 dây SDA/SCL sát nhau để giảm nhiễu. |
| | I2C SDA (5x VLX + LMS) | Pin 24 (GPIO26) | Giao tiếp tốc độ cao. |
| **Ngắt Cảm Biến ToF** | XSHUT 0 (Mắt Giữa) | Pin 25 (GPIO27) | Multiplexing phần cứng. |
| | XSHUT 1 (Mắt Trái) | Pin 26 (GPIO14) | Multiplexing phần cứng. |
| | XSHUT 2 (Mắt Phải) | Pin 28 (GPIO13) | Multiplexing phần cứng. |
| | XSHUT 3 (Sườn Trái) | Pin 6 (GPIO16/RX2) | **BẮT BUỘC:** Gom 2 dây XSHUT sườn sát nhau. |
| | XSHUT 4 (Sườn Phải) | Pin 7 (GPIO17/TX2) | Gom cùng XSHUT 3. |
| **Điều Khiển Động Cơ** | LPWM Phải (Right Motor) | Pin 9 (GPIO18) | Cấp tín hiệu cho BTS7960 phải. |
| | RPWM Phải (Right Motor) | Pin 10 (GPIO19) | Cấp tín hiệu cho BTS7960 phải. |
| | LPWM Trái (Left Motor) | Pin 11 (GPIO21) | Cấp tín hiệu cho BTS7960 trái. |
| | RPWM Trái (Left Motor) | Pin 14 (GPIO22) | Cấp tín hiệu cho BTS7960 trái. |
| **Giao Diện Người Dùng** | Nút bấm cảm ứng (TTP223) | Pin 5 (GPIO4) | Khởi động FSM & Start Delay 3s. |
| | I2C SDA (OLED 0.96) | Pin 15 (GPIO23) | Bus I2C riêng biệt tránh nghẽn luồng cảm biến. |
| | I2C SCL (OLED 0.96) | Pin 8 (GPIO5) | |
| **Cấu Hình Bắt Buộc** | Chân CS của module LMS | -> Nối nguồn 3.3V | Định cấu hình địa chỉ I2C cho IMU. |
| | Chân SA0 của module LMS | -> Nối nguồn 3.3V | Định cấu hình địa chỉ I2C cho IMU. |
| **CHÂN CẤM SỬ DỤNG** | PIN 27 (GPIO12) | **BỎ QUA** | **TUYỆT ĐỐI KHÔNG SỬ DỤNG.** Gây lỗi Boot. |
| | Pins 12 & 13 (RX0, TX0) | **BỎ QUA** | Dành riêng cho UART nạp code / Debug USB. |

Sơ đồ nối dây tổng thể và các đi-ốt bảo vệ mạch đều được cung cấp cụ thể dưới dạng file thiết kế đồ họa trong thư mục /Documents (Fritzing).
