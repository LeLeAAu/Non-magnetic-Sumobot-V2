// DEBUG 1 TCRT5000

#define PIN_TCRT 32
const uint16_t THRESHOLD_WHITE = 900;

void setup() {
  Serial.begin(115200);
  pinMode(PIN_TCRT, INPUT);
}

void loop() {
  uint16_t val = analogRead(PIN_TCRT);

  Serial.print("TCRT: ");
  Serial.print(val);

  if(val >= THRESHOLD_WHITE){
    Serial.print("  <<< WHITE");
  }

  Serial.println();

  delay(50);
}