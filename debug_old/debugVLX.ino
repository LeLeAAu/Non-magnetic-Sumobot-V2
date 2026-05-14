#include <Wire.h>
#include <VL53L1X.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// --- PIN MAPPING ---
#define I2C_SDA 26
#define I2C_SCL 25
const int XSHUT_PINS[5] = {27, 14, 13, 16, 17}; 
const uint8_t VLX_ADDRESSES[5] = {0x30, 0x31, 0x32, 0x33, 0x34};

#define OLED_SDA 23
#define OLED_SCL 5
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define SCREEN_ADDRESS 0x3C

TwoWire I2COLED = TwoWire(1);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &I2COLED, -1);

VL53L1X sensorsToF[5];
uint16_t dist[5] = {8190, 8190, 8190, 8190, 8190};

void setup() {
    Serial.begin(115200);
    delay(1000); // Chờ Serial ổn định
    Serial.println("\n=== TOF MULTI-SENSOR DEBUG BOOT ===");

    // 1. KHỞI TẠO OLED
    I2COLED.begin(OLED_SDA, OLED_SCL, 100000); 
    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println("❌ OLED Fail!");
        while (1);
    }
    
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("System Booting...");
    display.display();

    // 2. KHỞI TẠO TOF BUS
    // Hạ xuống 100kHz để "vượt qua" nhiễu dây rời/breadboard
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(100000); 

    // Reset vật lý: Tắt hết tất cả các mắt
    for (int i = 0; i < 5; i++) {
        pinMode(XSHUT_PINS[i], OUTPUT);
        digitalWrite(XSHUT_PINS[i], LOW);
    }
    delay(100); // Chờ điện áp xả sạch

    // 3. BOOT TUẦN TỰ (VỚI KIỂM TRA ĐỊA CHỈ 0x29)
    for (int i = 0; i < 5; i++) {
        Serial.printf("--- Booting Sensor %d (Pin %d) ---\n", i, XSHUT_PINS[i]);
        
        digitalWrite(XSHUT_PINS[i], HIGH);
        delay(100); // CHO CHIP THỜI GIAN KHỞI ĐỘNG (Rất quan trọng)

        // Kiểm tra xem chip có tồn tại trên Bus I2C không
        Wire.beginTransmission(0x29);
        if (Wire.endTransmission() != 0) {
            Serial.printf("❌ Không thấy 0x29 tại Pin %d. Kiểm tra dây/nguồn!\n", XSHUT_PINS[i]);
            display.clearDisplay();
            display.printf("ERR: Pin %d\nNo 0x29 found", XSHUT_PINS[i]);
            display.display();
            while (1) delay(10);
        }

        // Nếu thấy 0x29, bắt đầu Init
        sensorsToF[i].setTimeout(500);
        if (!sensorsToF[i].init()) {
            Serial.printf("❌ Init FAIL tại Pin %d. Library mismatch?\n", XSHUT_PINS[i]);
            display.clearDisplay();
            display.printf("ERR: Pin %d\nInit Failed", XSHUT_PINS[i]);
            display.display();
            while (1) delay(10);
        }

        // Đổi địa chỉ
        sensorsToF[i].setAddress(VLX_ADDRESSES[i]);
        
        // Cấu hình tối ưu cho Sumo Bot (Phản xạ nhanh)
        sensorsToF[i].setDistanceMode(VL53L1X::Medium); // Medium cân bằng giữa tốc độ và khoảng cách
        sensorsToF[i].setMeasurementTimingBudget(33000); 
        sensorsToF[i].startContinuous(33); // Đo liên tục mỗi 33ms

        Serial.printf("✅ Sensor %d OK -> Addr: 0x%02X\n", i, VLX_ADDRESSES[i]);
        display.printf("ToF %d: OK\n", i);
        display.display();
    }

    Serial.println(">>> ALL SENSORS READY!");
    // Sau khi boot xong xuôi, có thể nâng tốc độ I2C lên 400k để loop mượt hơn
    Wire.setClock(400000); 
}

void loop() {
    static uint8_t tick = 0;
    tick++;

    // Đọc dữ liệu phi tập trung (Non-blocking check)
    for (int i = 0; i < 5; i++) {
        if (sensorsToF[i].dataReady()) {
            dist[i] = sensorsToF[i].read(false);
        }
        
        if (sensorsToF[i].timeoutOccurred()) {
            dist[i] = 8190;
        }
    }

    // In Serial để debug nhanh
    Serial.printf("C:%4d L:%4d R:%4d SL:%4d SR:%4d\n", 
                  dist[0], dist[1], dist[2], dist[3], dist[4]);

    // Vẽ OLED
    display.clearDisplay();
    display.setCursor(0, 0);
    display.printf("L:%-4d C:%-4d R:%-4d", dist[1], dist[0], dist[2]);
    display.drawLine(0, 11, 128, 11, SSD1306_WHITE);
    display.setCursor(0, 15);
    display.printf("SL:%-4d    SR:%-4d", dist[3], dist[4]);

    // Heartbeat indicator
    if (tick % 2 == 0) display.fillRect(124, 28, 4, 4, SSD1306_WHITE);
    
    display.display();
    delay(20); 
}