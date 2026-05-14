// --- KHAI BÁO CHÂN (Từ code gốc) ---
#define PIN_TCRT_DETECT 39
#define PIN_TCRT_BL 34
#define PIN_TCRT_BR 35
#define PIN_TCRT_FL 32
#define PIN_TCRT_FR 33

// --- NGƯỠNG CALIB (Từ code gốc) ---
const uint16_t TCRT_EDGE_TH = 500;   // Dưới 500 là vạch trắng
const uint16_t TCRT_LIFT_TH = 4000;  // Dưới 4000 là có bụng địch đè lên

void setup() {
  // Trùng với baudrate trong file sumoInit.ino của bạn
  Serial.begin(115200);
  
  // Khởi tạo pin mode
  pinMode(PIN_TCRT_FL, INPUT);
  pinMode(PIN_TCRT_FR, INPUT);
  pinMode(PIN_TCRT_BL, INPUT);
  pinMode(PIN_TCRT_BR, INPUT);
  pinMode(PIN_TCRT_DETECT, INPUT);

  Serial.println(">>> BAT DAU TEST 5 CAM BIEN TCRT5000 <<<");
  Serial.println("--------------------------------------------------");
}

uint16_t readADC(int pin) {
    analogRead(pin); // Đọc bỏ lần đầu để ổn định điện áp
    delayMicroseconds(10); 
    return analogRead(pin); // Lấy giá trị lần này
}

void loop() {
  // 1. Đọc giá trị ADC thô
  uint16_t val_fl = readADC(PIN_TCRT_FL);
  uint16_t val_fr = readADC(PIN_TCRT_FL);
  uint16_t val_bl = readADC(PIN_TCRT_FL);
  uint16_t val_br = readADC(PIN_TCRT_FL);
  uint16_t val_detect = analogRead(PIN_TCRT_DETECT);

  // 2. In giá trị của 2 mắt mũi (Front)
  Serial.print("Mui Trai(FL): "); Serial.print(val_fl);
  Serial.print(val_fl <= TCRT_EDGE_TH ? "\t[TRANG] | " : "\t[DEN]   | ");
  
  Serial.print("Mui Phai(FR): "); Serial.print(val_fr);
  Serial.print(val_fr <= TCRT_EDGE_TH ? "\t[TRANG] | " : "\t[DEN]   | ");

  // 3. In giá trị của 2 mắt đuôi (Back)
  Serial.print("Duoi Trai(BL): "); Serial.print(val_bl);
  Serial.print(val_bl <= TCRT_EDGE_TH ? "\t[TRANG] | " : "\t[DEN]   | ");

  Serial.print("Duoi Phai(BR): "); Serial.print(val_br);
  Serial.print(val_br <= TCRT_EDGE_TH ? "\t[TRANG] | " : "\t[DEN]   | ");

  // 4. In giá trị của mắt bụng (Detect Lift)
  Serial.print("Bung(DETECT): "); Serial.print(val_detect);
  Serial.print(val_detect <= TCRT_LIFT_TH ? "\t[BI DE/XUC]" : "\t[TRONG]");

  Serial.println(); // Xuống dòng cho frame tiếp theo
  
  // Delay nhẹ để dễ nhìn trên Serial Monitor
  delay(150);
}