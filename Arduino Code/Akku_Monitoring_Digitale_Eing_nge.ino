const int CURRENT_PIN_1 = A0;
const int CURRENT_PIN_2 = A1;
const int TEMP_PIN_LEFT   = A2;
const int TEMP_PIN_CENTER = A3;
const int TEMP_PIN_RIGHT  = A4;
const unsigned long CURRENT_INTERVAL_MS = 500;
const unsigned long PIN_INTERVAL_MS = 10000;
unsigned long lastCurrentSend = 0;
unsigned long lastTempSend = 0;

float readVoltage(int pin) {
  int raw = analogRead(pin);
  return raw * (5.0 / 1023.0);
}

void setup() {
  Serial.begin(115200);
  pinMode(CURRENT_PIN_1, INPUT);
  pinMode(CURRENT_PIN_2, INPUT);
  pinMode(TEMP_PIN_LEFT, INPUT);
  pinMode(TEMP_PIN_CENTER, INPUT);
  pinMode(TEMP_PIN_RIGHT, INPUT);
  delay(1000);
  Serial.println("ARDUINO_BATTERY_MONITOR_START");
}

void loop() {
  unsigned long now = millis();
  if (now - lastCurrentSend >= CURRENT_INTERVAL_MS) {
    lastCurrentSend = now;
    float currentVoltage1 = readVoltage(CURRENT_PIN_1);
    float currentVoltage2 = readVoltage(CURRENT_PIN_2);
    Serial.print("CURRENT;");
    Serial.print("a0=");
    Serial.print(currentVoltage1, 3);
    Serial.print(";a1=");
    Serial.println(currentVoltage2, 3);
  }

  if (now - lastTempSend >= PIN_INTERVAL_MS) {
    lastTempSend = now;
    float tempLeftVoltage = readVoltage(TEMP_PIN_LEFT);
    float tempCenterVoltage = readVoltage(TEMP_PIN_CENTER);
    float tempRightVoltage = readVoltage(TEMP_PIN_RIGHT);
    Serial.print("TEMP;");
    Serial.print("left=");
    Serial.print(tempLeftVoltage, 3);
    Serial.print(";center=");
    Serial.print(tempCenterVoltage, 3);
    Serial.print(";right=");
    Serial.println(tempRightVoltage, 3);
  }
}
