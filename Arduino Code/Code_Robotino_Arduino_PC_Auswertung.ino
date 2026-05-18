/*
  Akku Monitoring Robotino / Arduino UNO
  Version: PC-berechnungsfreundlich

  Änderungen gegenüber der langen Version:
  - Keine Auto-Kalibrierung beim Start, weil beim Start bereits Last anliegt.
  - Nur EIN Strompfad / EIN Hauptstromsensor wird ausgewertet.
  - Kein Average und kein Verbrauch seit Start auf dem Arduino.
    Der PC bekommt I und dt_ms und kann daraus Average/SOC/Verbrauch berechnen.
  - Temperatur wird wie im Chat besprochen mit internem Pullup gelesen.
    Annahme: Temperaturleitungen sind passive NTC/PTC-Fühler gegen GND.
  - Plug-In-Zustände werden für alle 6 Akkus einzeln gezählt.
  - Plug-In wird nicht ständig schnell abgefragt, sondern beim Start stabil eingelesen
    und danach nur gelegentlich neu gescannt.
*/

// ------------------------------------------------------------
// PINBELEGUNG
// ------------------------------------------------------------

// Ein Hauptstromsensor auf dem gemeinsamen Strompfad
const int CURRENT_PIN = A0;

// Optionaler zweiter Current-Pin wird NICHT zur Stromberechnung genutzt.
// Du kannst ihn für Debugging angeschlossen lassen, aber er wird ignoriert.
// const int CURRENT_PIN_UNUSED = A1;

// Temperatur je Doppelakku / Akkugruppe
const int TEMP_PIN_LEFT   = A2;
const int TEMP_PIN_CENTER = A3;
const int TEMP_PIN_RIGHT  = A4;

// Plug-In Zustände für 6 einzelne Akkus
const int PLUG_PIN_RIGHTUP    = 7;
const int PLUG_PIN_RIGHTDO    = 6;
const int PLUG_PIN_CENTERUP   = 5;
const int PLUG_PIN_CENTERDO   = 4;
const int PLUG_PIN_LEFTUP     = 3;
const int PLUG_PIN_LEFTDO     = 2;

// ------------------------------------------------------------
// EINSTELLUNGEN
// ------------------------------------------------------------

const float VCC = 5.0;
const float ADC_MAX = 1023.0;

// ACS758ECB-200B: ca. 10 mV/A = 0.010 V/A
// Falls ihr einen anderen ACS758-Typ habt, muss dieser Wert angepasst werden.
const float ACS758_SENSITIVITY_V_PER_A = 0.010;

// Stromsensor-Nullpunkt.
// WICHTIG: Keine Auto-Kalibrierung beim Start, weil beim Start bereits Strom fließt.
// Diesen Wert später einmal möglichst bei 0 A messen und hier eintragen.
// Typisch ist ungefähr 2.500 V.
const float CURRENT_ZERO_OFFSET_V = 2.500;

// Falls der Strom falsch herum angezeigt wird, auf -1.0 setzen.
const float CURRENT_DIRECTION = 1.0;

// Kleine Stromwerte unterhalb dieser Grenze werden als 0 A behandelt.
// Das reduziert Offset-Rauschen um den Nullpunkt herum.
const float CURRENT_CUTOFF_A = 0.5;

// Kapazität pro Akku
const float CAPACITY_PER_BATTERY_AH = 4.0;

// Plug-In Logik:
// Bei INPUT_PULLUP gilt normalerweise:
// offen = HIGH = 1
// gegen GND = LOW = 0
// Wenn Akku/Signal bei LOW als "eingesteckt" erkannt werden soll, true lassen.
const bool PLUG_ACTIVE_LOW = true;

// Temperatur:
// Nach unserer Analyse sind die Temperaturleitungen sehr wahrscheinlich passive
// Temperaturfühler gegen GND. Deshalb nutzen wir den internen Pullup als
// einfachen Spannungsteiler.
const bool USE_INTERNAL_PULLUP_FOR_TEMP = true;

// Messintervalle
const unsigned long CURRENT_INTERVAL_MS = 500;

// Vollständige DATA-Ausgabe an den PC.
// Der PC kann aus I_A und DT_MS Average, Ah-Verbrauch und SOC berechnen.
const unsigned long DATA_INTERVAL_MS = 1000;

// Plug-In wird beim Start gelesen und später nur gelegentlich neu gescannt.
// Falls die Akkus während des Betriebs fast nie gewechselt werden, reicht 60 s.
// Für "nur beim Start" diesen Wert sehr groß setzen, z. B. 3600000.
const unsigned long PLUG_RESCAN_INTERVAL_MS = 300000;

// Plug-Sampling: mehrere Messungen pro Scan, damit kurze Störungen nicht zählen.
const byte PLUG_SCAN_SAMPLES = 25;
const unsigned int PLUG_SCAN_DELAY_MS = 10;

// Temperatur-Averaging
const int TEMP_SAMPLES = 20;

// Strom-Averaging
const int CURRENT_SAMPLES = 20;

// ------------------------------------------------------------
// VARIABLEN
// ------------------------------------------------------------

unsigned long lastCurrentMs = 0;
unsigned long lastDataMs = 0;
unsigned long lastPlugScanMs = 0;

unsigned long lastCurrentDtMs = 0;

float currentVoltage_V = 0.0;
float current_A = 0.0;

float tempLeft_V = 0.0;
float tempCenter_V = 0.0;
float tempRight_V = 0.0;

// Reihenfolge:
// 0 leftup
// 1 leftdown
// 2 centerup
// 3 centerdown
// 4 rightup
// 5 rightdown
bool plugStates[6] = {false, false, false, false, false, false};

// ------------------------------------------------------------
// SETUP
// ------------------------------------------------------------

void setup()
{
  Serial.begin(115200);

  pinMode(CURRENT_PIN, INPUT);

  if (USE_INTERNAL_PULLUP_FOR_TEMP)
  {
    pinMode(TEMP_PIN_LEFT, INPUT_PULLUP);
    pinMode(TEMP_PIN_CENTER, INPUT_PULLUP);
    pinMode(TEMP_PIN_RIGHT, INPUT_PULLUP);
  }
  else
  {
    pinMode(TEMP_PIN_LEFT, INPUT);
    pinMode(TEMP_PIN_CENTER, INPUT);
    pinMode(TEMP_PIN_RIGHT, INPUT);
  }

  pinMode(PLUG_PIN_LEFTUP, INPUT_PULLUP);
  pinMode(PLUG_PIN_LEFTDO, INPUT_PULLUP);
  pinMode(PLUG_PIN_CENTERUP, INPUT_PULLUP);
  pinMode(PLUG_PIN_CENTERDO, INPUT_PULLUP);
  pinMode(PLUG_PIN_RIGHTUP, INPUT_PULLUP);
  pinMode(PLUG_PIN_RIGHTDO, INPUT_PULLUP);

  delay(1000);

  scanPlugStates();
  readTemperatures();
  readCurrentSensor();

  lastCurrentMs = millis();
  lastDataMs = millis();
  lastPlugScanMs = millis();

  Serial.println(F("START;AkkuMonitoring=ready"));
  Serial.print(F("CONFIG;currentPin=A0;offsetV="));
  Serial.print(CURRENT_ZERO_OFFSET_V, 3);
  Serial.print(F(";sensitivityVperA="));
  Serial.print(ACS758_SENSITIVITY_V_PER_A, 3);
  Serial.print(F(";capacityPerBatteryAh="));
  Serial.println(CAPACITY_PER_BATTERY_AH, 2);

  printDataLine(0);
}

// ------------------------------------------------------------
// LOOP
// ------------------------------------------------------------

void loop()
{
  unsigned long now = millis();

  // Strom messen
  if (now - lastCurrentMs >= CURRENT_INTERVAL_MS)
  {
    lastCurrentDtMs = now - lastCurrentMs;
    lastCurrentMs = now;

    readCurrentSensor();
  }

  // Plug-In gelegentlich neu einlesen
  if (now - lastPlugScanMs >= PLUG_RESCAN_INTERVAL_MS)
  {
    lastPlugScanMs = now;
    scanPlugStates();
  }

  // Vollständige Daten für den PC ausgeben
  if (now - lastDataMs >= DATA_INTERVAL_MS)
  {
    unsigned long dataDtMs = now - lastDataMs;
    lastDataMs = now;

    readTemperatures();
    printDataLine(dataDtMs);
  }
}

// ------------------------------------------------------------
// ANALOG EINLESEN
// ------------------------------------------------------------

float readVoltageAverage(int pin, int samples)
{
  long sum = 0;

  // Erste Messung nach Kanalwechsel verwerfen
  analogRead(pin);
  delayMicroseconds(300);

  for (int i = 0; i < samples; i++)
  {
    sum += analogRead(pin);
    delayMicroseconds(300);
  }

  float rawAverage = sum / (float)samples;
  return rawAverage * (VCC / ADC_MAX);
}

// ------------------------------------------------------------
// STROM
// ------------------------------------------------------------

void readCurrentSensor()
{
  currentVoltage_V = readVoltageAverage(CURRENT_PIN, CURRENT_SAMPLES);

  current_A =
    ((currentVoltage_V - CURRENT_ZERO_OFFSET_V) / ACS758_SENSITIVITY_V_PER_A)
    * CURRENT_DIRECTION;

  if (absFloat(current_A) < CURRENT_CUTOFF_A)
  {
    current_A = 0.0;
  }
}

// ------------------------------------------------------------
// TEMPERATUR
// ------------------------------------------------------------

float readTemperatureVoltage(int pin)
{
  if (USE_INTERNAL_PULLUP_FOR_TEMP)
  {
    // Interner Pullup bildet mit dem NTC/PTC gegen GND einen Spannungsteiler.
    pinMode(pin, INPUT_PULLUP);
    delay(3);
  }

  return readVoltageAverage(pin, TEMP_SAMPLES);
}

void readTemperatures()
{
  tempLeft_V = readTemperatureVoltage(TEMP_PIN_LEFT);
  tempCenter_V = readTemperatureVoltage(TEMP_PIN_CENTER);
  tempRight_V = readTemperatureVoltage(TEMP_PIN_RIGHT);
}

// ------------------------------------------------------------
// PLUG-IN
// ------------------------------------------------------------

bool readPlugConnectedRaw(int pin)
{
  int raw = digitalRead(pin);

  if (PLUG_ACTIVE_LOW)
  {
    return raw == LOW;
  }
  else
  {
    return raw == HIGH;
  }
}

bool readPlugStableByMajority(int pin)
{
  byte connectedCount = 0;

  for (byte i = 0; i < PLUG_SCAN_SAMPLES; i++)
  {
    if (readPlugConnectedRaw(pin))
    {
      connectedCount++;
    }

    delay(PLUG_SCAN_DELAY_MS);
  }

  // Mehrheit entscheidet
  return connectedCount > (PLUG_SCAN_SAMPLES / 2);
}

void scanPlugStates()
{
  plugStates[0] = readPlugStableByMajority(PLUG_PIN_LEFTUP);
  plugStates[1] = readPlugStableByMajority(PLUG_PIN_LEFTDO);
  plugStates[2] = readPlugStableByMajority(PLUG_PIN_CENTERUP);
  plugStates[3] = readPlugStableByMajority(PLUG_PIN_CENTERDO);
  plugStates[4] = readPlugStableByMajority(PLUG_PIN_RIGHTUP);
  plugStates[5] = readPlugStableByMajority(PLUG_PIN_RIGHTDO);
}

int getBatteryCount()
{
  int count = 0;

  for (byte i = 0; i < 6; i++)
  {
    if (plugStates[i])
    {
      count++;
    }
  }

  return count;
}

float getTotalCapacityAh()
{
  return getBatteryCount() * CAPACITY_PER_BATTERY_AH;
}

// ------------------------------------------------------------
// AUSGABE
// ------------------------------------------------------------

void printDataLine(unsigned long dataDtMs)
{
  int batteryCount = getBatteryCount();
  float totalCapacity_Ah = batteryCount * CAPACITY_PER_BATTERY_AH;

  Serial.print(F("DATA;ms="));
  Serial.print(millis());

  Serial.print(F(";dt_ms="));
  Serial.print(dataDtMs);

  Serial.print(F(";current_v="));
  Serial.print(currentVoltage_V, 3);

  Serial.print(F(";current_a="));
  Serial.print(current_A, 2);

  Serial.print(F(";temp_left_v="));
  Serial.print(tempLeft_V, 3);

  Serial.print(F(";temp_center_v="));
  Serial.print(tempCenter_V, 3);

  Serial.print(F(";temp_right_v="));
  Serial.print(tempRight_V, 3);

  Serial.print(F(";leftup="));
  Serial.print(plugStates[0]);

  Serial.print(F(";leftdown="));
  Serial.print(plugStates[1]);

  Serial.print(F(";centerup="));
  Serial.print(plugStates[2]);

  Serial.print(F(";centerdown="));
  Serial.print(plugStates[3]);

  Serial.print(F(";rightup="));
  Serial.print(plugStates[4]);

  Serial.print(F(";rightdown="));
  Serial.print(plugStates[5]);

  Serial.print(F(";battery_count="));
  Serial.print(batteryCount);

  Serial.print(F(";capacity_ah="));
  Serial.print(totalCapacity_Ah, 2);

  Serial.print(F(";who="));
  printWhoInline();

  Serial.println();
}

void printWhoInline()
{
  bool first = true;

  if (plugStates[0])
  {
    Serial.print(F("leftup"));
    first = false;
  }

  if (plugStates[1])
  {
    if (!first) Serial.print(F(","));
    Serial.print(F("leftdown"));
    first = false;
  }

  if (plugStates[2])
  {
    if (!first) Serial.print(F(","));
    Serial.print(F("centerup"));
    first = false;
  }

  if (plugStates[3])
  {
    if (!first) Serial.print(F(","));
    Serial.print(F("centerdown"));
    first = false;
  }

  if (plugStates[4])
  {
    if (!first) Serial.print(F(","));
    Serial.print(F("rightup"));
    first = false;
  }

  if (plugStates[5])
  {
    if (!first) Serial.print(F(","));
    Serial.print(F("rightdown"));
    first = false;
  }

  if (first)
  {
    Serial.print(F("none"));
  }
}

// ------------------------------------------------------------
// HILFSFUNKTION
// ------------------------------------------------------------

float absFloat(float value)
{
  if (value < 0.0)
  {
    return -value;
  }

  return value;
}
