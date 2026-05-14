#include <Wire.h>
#include <VL53L1X.h>
#include <SparkFunLSM6DS3.h>
#include <math.h>

// --- MAPPING PIN (Giữ nguyên theo code của bạn) ---
#define I2C_SDA 26
#define I2C_SCL 25
const int XSHUT_PINS[5] = {27, 14, 13, 16, 17}; // Giữa, Trái, Phải, Sườn Trái, Sườn Phải
const uint8_t VLX_ADDRESSES[5] = {0x30, 0x31, 0x32, 0x33, 0x34};

VL53L1X sensorsToF[5];
LSM6DS3 myIMU(I2C_MODE, 0x6B);

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("\n--- HE THONG DEBUG SUMOBOT: VLX + LSM ---");

  // 1. Khoi tao I2C cho cam bien
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  // 2. Khoi tao IMU
  if (myIMU.begin() != 0) {
    Serial.println("[ERROR] LSM6DS3 fail!");
  } else {
    Serial.println("[OK] LSM6DS3 ready.");
  }

  // 3. Quy trinh gan dia chi dong cho 5x VL53L1X
  Serial.println("Dang khoi tao VL53L1X...");
  for (int i = 0; i < 5; i++) {
    pinMode(XSHUT_PINS[i], OUTPUT);
    digitalWrite(XSHUT_PINS[i], LOW);
  }
  delay(100);

  for (int i = 0; i < 5; i++) {
    digitalWrite(XSHUT_PINS[i], HIGH);
    delay(20); // Cho thoi gian chip boot
    sensorsToF[i].setTimeout(50);
    if (!sensorsToF[i].init()) {
      Serial.printf("[ERROR] VLX %d (Pin %d) fail!\n", i, XSHUT_PINS[i]);
    } else {
      sensorsToF[i].setAddress(VLX_ADDRESSES[i]);
      sensorsToF[i].setDistanceMode(VL53L1X::Long);
      sensorsToF[i].setMeasurementTimingBudget(33000);
      sensorsToF[i].startContinuous(38);
      Serial.printf("[OK] VLX %d ready at 0x%02X\n", i, VLX_ADDRESSES[i]);
    }
  }
  Serial.println("------------------------------------------");
}

void loop() {
  uint16_t d[5];
  
  for (int i = 0; i < 5; i++) {
    // CHỈ ĐỌC KHI SENSOR BÁO CÓ DỮ LIỆU MỚI
    if (sensorsToF[i].dataReady()) {
      d[i] = sensorsToF[i].read(false); 
    } else {
      // Nếu chưa có data mới, giữ nguyên giá trị cũ hoặc đánh dấu để debug
      // Đừng gán bằng 0 ở đây để tránh bị nhảy số ảo
    }
    delayMicroseconds(500); // Nghỉ một chút giữa các lần gọi I2C cho bus ổn định
  }

  // --- IMU ---
  float ax = myIMU.readFloatAccelX();
  float ay = myIMU.readFloatAccelY();
  float az = myIMU.readFloatAccelZ();
  float pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI;
  float roll  = atan2(ay, az) * 180.0 / M_PI;

  // In log rút gọn để check SL/SR
  Serial.printf("ToF: M:%4d L:%4d R:%4d SL:%4d SR:%4d | P:%5.1f R:%5.1f\n", 
                d[0], d[1], d[2], d[3], d[4], pitch, roll);

  delay(40); // Khớp với rate 38ms của VLX
}