/***************************************************************************
 * Arduino_Code_Akku_Monitoring_Standalone_1Akku_1Temp_TempVoltage.ino
 *
 * Standalone-Akku-Monitoring für Tests am Ubuntu-Rechner / Arduino.
 *
 * Diese Version ist für den vereinfachten Testaufbau gedacht:
 * - 1 Akkuhalter / 1 Akku
 * - 1 Strompfad mit ACS758
 * - 1 Temperaturmesswert
 * - Spannung wird im Code gesetzt oder per USB/Serial empfangen
 * - Durchschnittsstrom über die letzten 20 Messwerte
 * - Coulomb Counting direkt auf dem Arduino
 * - Temperaturabhängigkeit der Spannung wird berücksichtigt und ausgegeben
 *
 * Beispiel für Spannung vom PC senden:
 *   VOLTAGE;20.10
 *   V=20.10
 *   SET_VOLTAGE=20.10
 *
 * Beispielausgabe:
 * DATA;ms=12345;dt_ms=1000;voltage_v=20.10;voltage_temp_correction_v=0.00;voltage_corrected_v=20.10;voltage_percent=87.88;current_v=2.443;current_a=0.00;current_avg_a=0.00;consumed_ah=0.0000;remaining_ah=3.5152;remaining_percent=87.88;runtime_remaining_min=-1.0;temp_v=3.700;temp_c=25.0;temp_valid=1;battery_count=1;capacity_ah=4.00;battery_mode=fixed_1_battery
 *
 ****************************************************************************/

// ------------------------------------------------------------
// PINBELEGUNG
// ------------------------------------------------------------

// ACS758-Ausgang. OUT vom ACS758 muss mit diesem Arduino-Pin verbunden sein.
const int CURRENT_PIN = A1;

// Nur noch ein Temperaturkanal, weil nur noch ein Akkuhalter genutzt wird.
// Falls dein Temperatursensor an einem anderen Analogpin hängt, hier ändern.
const int TEMP_PIN = A2;

// Akkuspannung über Spannungsteiler auf A3.
// Der Messwert an A3 wird mit dem Teilerfaktor 4.7 zurückgerechnet.
const int VOLTAGE_PIN = A3;


// ------------------------------------------------------------
// Variablen
// ------------------------------------------------------------

// Nullpunkt des Stromsensors in Volt.
// Wichtig: Dieser Wert muss zu deinem Aufbau passen.
// Wenn bei ausgeschaltetem Verbraucher z. B. current_v=2.443 V ist,
// dann sollte hier ungefähr 2.443 stehen.
const float CURRENT_ZERO_OFFSET_V = 2.433;

// Standard-Spannung, falls nichts per Serial gesendet wird.
// Diese Spannung kannst du zum Testen einfach im Code ändern.
float batteryVoltage_V = 19.0;

// Effektiver Pullup-Widerstand.
// Interne Pullups sind ungenau. Bei euren bisherigen 3.7-V-Messwerten ist
// ungefähr 3.3 kOhm als Startwert plausibel.
const float TEMP_PULLUP_OHM = 3300.0;

// ------------------------------------------------------------
// ALLGEMEINE ADC-EINSTELLUNGEN
// ------------------------------------------------------------

// Arduino UNO misst analog standardmäßig gegen 5 V.
const float VCC = 5.0;

// 10-bit ADC: analogRead liefert 0 bis 1023.
const float ADC_MAX = 1023.0;

// ------------------------------------------------------------
// SPANNUNGSMESSUNG ÜBER SPANNUNGSTEILER
// ------------------------------------------------------------

// Faktor des Spannungsteilers.
// Beispiel: Wenn an A3 4.00 V anliegen, wird daraus 4.00 * 4.7 = 18.80 V.
const float VOLTAGE_DIVIDER_FACTOR = 4.7;

// Anzahl ADC-Samples pro Spannungsmessung.
const int VOLTAGE_ADC_SAMPLES = 20;

// ------------------------------------------------------------
// STROMMESSUNG ACS758
// ------------------------------------------------------------

// ACS758-200B: ungefähr 10 mV/A.
// Falls euer Sensor nicht die 200B-Variante ist, muss dieser Wert angepasst werden.
const float ACS758_SENSITIVITY_V_PER_A = 0.010;

// Stromrichtung.
// Falls Verbrauch negativ angezeigt wird, auf -1.0 setzen.
// Falls Verbrauch positiv angezeigt wird, auf 1.0 lassen.
const float CURRENT_DIRECTION = -1.0;

// Kleine Ströme werden als 0 A behandelt.
// Das verhindert, dass Offsetrauschen als Verbrauch gezählt wird.
const float CURRENT_CUTOFF_A = 0.50;

// Wie oft der Stromsensor gemessen wird.
// 200 ms bedeutet 5 Messungen pro Sekunde.
const unsigned long CURRENT_SAMPLE_INTERVAL_MS = 200;

// Anzahl der letzten Strommesswerte für den gleitenden Durchschnitt.
// 20 Messwerte * 200 ms = Durchschnitt über ca. 4 Sekunden.
const int CURRENT_AVG_WINDOW = 20;

// Anzahl ADC-Samples pro Strommessung.
// Mehr Samples = stabiler, aber etwas langsamer.
const int CURRENT_ADC_SAMPLES = 20;

// ------------------------------------------------------------
// SPANNUNG UND AKKU
// ------------------------------------------------------------

// Akkuzahl fest auf 1.
const int FIXED_BATTERY_COUNT = 1;

// Kapazität pro Akku.
const float CAPACITY_PER_BATTERY_AH = 4.0;

// Gesamtkapazität.
const float TOTAL_CAPACITY_AH = FIXED_BATTERY_COUNT * CAPACITY_PER_BATTERY_AH;

// Spannung bei "voll".
// Für viele 18-V-Werkzeugakkus liegt voll ungefähr bei 20.5 V.
// Den Wert kannst du später genauer kalibrieren.
const float FULL_VOLTAGE_V = 20.50;

// Spannung bei "leer / kritisch".
// Dieser Wert ist geschätzt und sollte später an eure Akkus angepasst werden.
const float EMPTY_VOLTAGE_V = 17.20;

// Spannungskorrektur nur nutzen, wenn der Durchschnittsstrom klein ist.
const float IDLE_CURRENT_THRESHOLD_A = 0.50;

// So lange muss der Strom unter dem Schwellwert sein, bevor Spannung
// zur Korrektur der Restladung verwendet wird.
const unsigned long IDLE_REQUIRED_MS = 20000;

// Mischfaktor für Coulomb Counting + Spannung.
// 0.8 bedeutet:
// 80 % bisherige Coulomb-Counting-Schätzung
// 20 % spannungsbasierte Schätzung
const float ALPHA_COULOMB = 0.75;

// ------------------------------------------------------------
// TEMPERATUR UND TEMPERATURABHÄNGIGE SPANNUNGSKORREKTUR
// ------------------------------------------------------------

// Wir nutzen den internen Pullup als einfache Testlösung.
// Das passt, wenn die Temperaturleitung ein passiver NTC/PTC gegen GND ist.
const bool USE_INTERNAL_PULLUP_FOR_TEMP = true;



// Annahme: 10k NTC bei 25 °C.
const float NTC_R0_OHM = 10000.0;
const float NTC_T0_K = 298.15;
const float NTC_BETA = 3950.0;

// Falls die Temperatur ungültig ist, verwenden wir diesen Wert.
const float TEMP_FALLBACK_C = 25.0;

// Gültigkeitsbereich der Spannung.
// Bei fast 5 V ist der Sensor wahrscheinlich offen.
// Bei fast 0 V ist er wahrscheinlich kurzgeschlossen.
const float TEMP_MIN_VALID_V = 0.10;
const float TEMP_MAX_VALID_V = 4.70;

// Plausibler Temperaturbereich.
const float TEMP_MIN_VALID_C = -20.0;
const float TEMP_MAX_VALID_C = 90.0;

// Anzahl ADC-Samples pro Temperaturmessung.
const int TEMP_ADC_SAMPLES = 20;

// Referenztemperatur für die Spannungskorrektur.
// Die Spannungs-SOC-Kurve wird gedanklich auf 25 °C bezogen.
const float TEMP_REFERENCE_C = 25.0;

// Temperaturkoeffizient der Akkuspannung.
// Startwert: 10 mV pro °C für den gesamten 18-V-Akku-Pack.
// Wenn der Akku kälter als 25 °C ist, wird die gemessene Spannung leicht nach oben korrigiert,
// weil kalte Akkus unter Last eher eine niedrigere Spannung zeigen.
// Wenn der Akku wärmer als 25 °C ist, wird leicht nach unten korrigiert.
const float VOLTAGE_TEMP_COEFF_V_PER_C = 0.010;

// ------------------------------------------------------------
// AUSGABE
// ------------------------------------------------------------

// DATA-Ausgabe jede Sekunde.
const unsigned long DATA_INTERVAL_MS = 1000;

// ------------------------------------------------------------
// LAUFZEITVARIABLEN STROM
// ------------------------------------------------------------

unsigned long lastCurrentSampleMs = 0;
unsigned long lastDataMs = 0;
unsigned long lastIntegrationMs = 0;
unsigned long idleStartMs = 0;

float currentVoltage_V = 0.0;
float current_A = 0.0;
float currentAvg_A = 0.0;

// Ringpuffer für die letzten 20 Stromwerte.
float currentBuffer[CURRENT_AVG_WINDOW];
int currentBufferIndex = 0;
int currentBufferCount = 0;

// Coulomb Counting.
float consumedAh = 0.0;
float remainingAh = TOTAL_CAPACITY_AH;

// Temperatur.
float tempVoltage_V = 0.0;
float temp_C = TEMP_FALLBACK_C;
bool tempValid = false;

// Serial-Empfang für optionale Spannungseingabe.
char serialBuffer[64];
int serialBufferIndex = 0;

// ------------------------------------------------------------
// SETUP
// ------------------------------------------------------------

void setup()
{
  Serial.begin(115200);

  pinMode(CURRENT_PIN, INPUT);
  pinMode(VOLTAGE_PIN, INPUT);

  if (USE_INTERNAL_PULLUP_FOR_TEMP)
  {
    pinMode(TEMP_PIN, INPUT_PULLUP);
  }
  else
  {
    pinMode(TEMP_PIN, INPUT);
  }

  // Ringpuffer mit 0 initialisieren.
  for (int i = 0; i < CURRENT_AVG_WINDOW; i++)
  {
    currentBuffer[i] = 0.0;
  }

  unsigned long now = millis();
  lastCurrentSampleMs = now;
  lastDataMs = now;
  lastIntegrationMs = now;
  idleStartMs = now;

  // Beim Start einmal Spannung und Temperatur lesen, damit die Spannungsschätzung
  // direkt die gemessene und temperaturkorrigierte Spannung nutzen kann.
  readBatteryVoltage();
  readTemperature();

  // Startwert für Restladung aus temperaturkorrigierter Spannung schätzen.
  remainingAh = voltageToRemainingAh(getTemperatureCorrectedVoltage());
  consumedAh = TOTAL_CAPACITY_AH - remainingAh;

  Serial.println(F("START;AkkuMonitoringStandalone=ready"));
  Serial.print(F("CONFIG;battery_count="));
  Serial.print(FIXED_BATTERY_COUNT);
  Serial.print(F(";capacity_ah="));
  Serial.print(TOTAL_CAPACITY_AH, 2);
  Serial.print(F(";voltage_pin=A3"));
  Serial.print(F(";voltage_divider_factor="));
  Serial.print(VOLTAGE_DIVIDER_FACTOR, 2);
  Serial.print(F(";current_offset_v="));
  Serial.print(CURRENT_ZERO_OFFSET_V, 3);
  Serial.print(F(";avg_window="));
  Serial.print(CURRENT_AVG_WINDOW);
  Serial.print(F(";temp_pin=A2"));
  Serial.print(F(";temp_voltage_coeff_v_per_c="));
  Serial.println(VOLTAGE_TEMP_COEFF_V_PER_C, 3);
}

// ------------------------------------------------------------
// LOOP
// ------------------------------------------------------------

void loop()
{
  unsigned long now = millis();

  // Optional: Spannung vom PC über USB/Serial empfangen.
  // Hinweis: Die reale Spannung wird unten zusätzlich über A3 gemessen
  // und überschreibt den Wert vor der Ausgabe.
  readSerialCommands();

  // Strom regelmäßig messen.
  if (now - lastCurrentSampleMs >= CURRENT_SAMPLE_INTERVAL_MS)
  {
    lastCurrentSampleMs = now;

    readCurrentSensor();

    // Für Durchschnitt, Verbrauch und Restlaufzeit nehmen wir den Betrag.
    // Dadurch wird Verbrauch positiv gezählt, auch wenn der Sensor anders herum eingebaut ist.
    addCurrentToAverage(absFloat(current_A));
    currentAvg_A = calculateCurrentAverage();

    updateIdleState(now);
  }

  // Coulomb Counting kontinuierlich berechnen.
  integrateConsumption(now);

  // Daten regelmäßig ausgeben.
  if (now - lastDataMs >= DATA_INTERVAL_MS)
  {
    unsigned long dtMs = now - lastDataMs;
    lastDataMs = now;

    // Spannung über A3 und einen Temperaturwert einlesen.
    readBatteryVoltage();
    readTemperature();

    // Wenn Stillstand erkannt wurde, darf die Spannungsschätzung den Coulomb-Counting-Wert korrigieren.
    // Dabei wird die temperaturkorrigierte Spannung genutzt.
    applyVoltageCorrectionIfIdle(now);

    printDataLine(dtMs);
  }
}

// ------------------------------------------------------------
// ANALOG EINLESEN
// ------------------------------------------------------------

float readVoltageAverage(int pin, int samples)
{
  long sum = 0;

  // Erste Messung nach Kanalwechsel verwerfen.
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
// STROMMESSUNG
// ------------------------------------------------------------

void readCurrentSensor()
{
  currentVoltage_V = readVoltageAverage(CURRENT_PIN, CURRENT_ADC_SAMPLES);

  current_A =
    ((currentVoltage_V - CURRENT_ZERO_OFFSET_V) / ACS758_SENSITIVITY_V_PER_A)
    * CURRENT_DIRECTION;

  if (absFloat(current_A) < CURRENT_CUTOFF_A)
  {
    current_A = 0.0;
  }
}

void addCurrentToAverage(float currentAbs_A)
{
  currentBuffer[currentBufferIndex] = currentAbs_A;

  currentBufferIndex++;

  if (currentBufferIndex >= CURRENT_AVG_WINDOW)
  {
    currentBufferIndex = 0;
  }

  if (currentBufferCount < CURRENT_AVG_WINDOW)
  {
    currentBufferCount++;
  }
}

float calculateCurrentAverage()
{
  if (currentBufferCount == 0)
  {
    return 0.0;
  }

  float sum = 0.0;

  for (int i = 0; i < currentBufferCount; i++)
  {
    sum += currentBuffer[i];
  }

  return sum / (float)currentBufferCount;
}

// ------------------------------------------------------------
// COULOMB COUNTING UND RESTLAUFZEIT
// ------------------------------------------------------------

void integrateConsumption(unsigned long now)
{
  unsigned long dtMs = now - lastIntegrationMs;
  lastIntegrationMs = now;

  // dt in Stunden.
  float dtHours = dtMs / 3600000.0;

  // Für Verbrauch nehmen wir den geglätteten Durchschnittsstrom.
  float usedAhInterval = currentAvg_A * dtHours;

  consumedAh += usedAhInterval;
  remainingAh -= usedAhInterval;

  // Grenzen setzen.
  if (remainingAh < 0.0)
  {
    remainingAh = 0.0;
  }

  if (remainingAh > TOTAL_CAPACITY_AH)
  {
    remainingAh = TOTAL_CAPACITY_AH;
  }

  consumedAh = TOTAL_CAPACITY_AH - remainingAh;
}

float calculateRuntimeRemainingMin()
{
  if (currentAvg_A < CURRENT_CUTOFF_A)
  {
    return -1.0; // -1 bedeutet: keine sinnvolle Restlaufzeit, weil kaum Strom fließt.
  }

  return (remainingAh / currentAvg_A) * 60.0;
}


// ------------------------------------------------------------
// SPANNUNGSMESSUNG
// ------------------------------------------------------------

float voltagePin_V = 0.0;

void readBatteryVoltage()
{
  // A3 misst nur die heruntergeteilte Spannung am Spannungsteiler.
  voltagePin_V = readVoltageAverage(VOLTAGE_PIN, VOLTAGE_ADC_SAMPLES);

  // Rückrechnung auf die echte Akkuspannung.
  batteryVoltage_V = voltagePin_V * VOLTAGE_DIVIDER_FACTOR;
}

// ------------------------------------------------------------
// SPANNUNGSBASIERTE SCHÄTZUNG MIT TEMPERATURABHÄNGIGKEIT
// ------------------------------------------------------------

float getVoltageTemperatureCorrection()
{
  // Wenn kein gültiger Temperaturwert vorhanden ist, korrigieren wir nicht.
  if (!tempValid)
  {
    return 0.0;
  }

  // Beispiel:
  // temp_C = 15 °C, Referenz = 25 °C
  // correction = 0.010 * (25 - 15) = +0.10 V
  // Die gemessene Spannung wird also leicht angehoben.
  return VOLTAGE_TEMP_COEFF_V_PER_C * (TEMP_REFERENCE_C - temp_C);
}

float getTemperatureCorrectedVoltage()
{
  return batteryVoltage_V + getVoltageTemperatureCorrection();
}

float voltageToPercent(float voltage_V)
{
  float percent =
    ((voltage_V - EMPTY_VOLTAGE_V) / (FULL_VOLTAGE_V - EMPTY_VOLTAGE_V)) * 100.0;

  if (percent < 0.0) percent = 0.0;
  if (percent > 100.0) percent = 100.0;

  return percent;
}

float voltageToRemainingAh(float voltage_V)
{
  return TOTAL_CAPACITY_AH * (voltageToPercent(voltage_V) / 100.0);
}

void updateIdleState(unsigned long now)
{
  if (currentAvg_A < IDLE_CURRENT_THRESHOLD_A)
  {
    // Wenn gerade erst Leerlauf erkannt wurde, Startzeit setzen.
    if (idleStartMs == 0)
    {
      idleStartMs = now;
    }
  }
  else
  {
    // Bei Last wird der Idle-Timer zurückgesetzt.
    idleStartMs = 0;
  }
}

bool isVoltageCorrectionAllowed(unsigned long now)
{
  if (idleStartMs == 0)
  {
    return false;
  }

  return (now - idleStartMs) >= IDLE_REQUIRED_MS;
}

void applyVoltageCorrectionIfIdle(unsigned long now)
{
  if (!isVoltageCorrectionAllowed(now))
  {
    return;
  }

  // Hier wird bewusst die temperaturkorrigierte Spannung genutzt.
  float voltageAh = voltageToRemainingAh(getTemperatureCorrectedVoltage());

  remainingAh =
    ALPHA_COULOMB * remainingAh +
    (1.0 - ALPHA_COULOMB) * voltageAh;

  if (remainingAh < 0.0) remainingAh = 0.0;
  if (remainingAh > TOTAL_CAPACITY_AH) remainingAh = TOTAL_CAPACITY_AH;

  consumedAh = TOTAL_CAPACITY_AH - remainingAh;
}

// ------------------------------------------------------------
// TEMPERATURMESSUNG: EIN TEMPERATURKANAL
// ------------------------------------------------------------

float readTemperatureVoltage()
{
  if (USE_INTERNAL_PULLUP_FOR_TEMP)
  {
    pinMode(TEMP_PIN, INPUT_PULLUP);
    delay(3);
  }

  return readVoltageAverage(TEMP_PIN, TEMP_ADC_SAMPLES);
}

bool isTemperatureVoltageValid(float voltage_V)
{
  return voltage_V >= TEMP_MIN_VALID_V && voltage_V <= TEMP_MAX_VALID_V;
}

float voltageToTemperatureC(float voltage_V)
{
  // Spannung außerhalb des sinnvollen Bereichs -> ungültig.
  if (!isTemperatureVoltageValid(voltage_V))
  {
    return TEMP_FALLBACK_C;
  }

  // Spannungsteiler:
  // 5V -> Pullup -> Analogpin -> NTC -> GND
  // U = VCC * R_NTC / (R_PULLUP + R_NTC)
  float ntcResistance_Ohm =
    TEMP_PULLUP_OHM * voltage_V / (VCC - voltage_V);

  if (ntcResistance_Ohm <= 0.0)
  {
    return TEMP_FALLBACK_C;
  }

  // Beta-Gleichung des NTC.
  float temp_K =
    1.0 / ((1.0 / NTC_T0_K) + (1.0 / NTC_BETA) * log(ntcResistance_Ohm / NTC_R0_OHM));

  float calculatedTemp_C = temp_K - 273.15;

  if (calculatedTemp_C < TEMP_MIN_VALID_C || calculatedTemp_C > TEMP_MAX_VALID_C)
  {
    return TEMP_FALLBACK_C;
  }

  return calculatedTemp_C;
}

void readTemperature()
{
  tempVoltage_V = readTemperatureVoltage();
  tempValid = isTemperatureVoltageValid(tempVoltage_V);

  if (tempValid)
  {
    temp_C = voltageToTemperatureC(tempVoltage_V);
  }
  else
  {
    temp_C = TEMP_FALLBACK_C;
  }
}

// ------------------------------------------------------------
// SERIAL INPUT: SPANNUNG VOM PC EINLESEN
// ------------------------------------------------------------

void readSerialCommands()
{
  while (Serial.available() > 0)
  {
    char c = Serial.read();

    if (c == '\n' || c == '\r')
    {
      serialBuffer[serialBufferIndex] = '\0';

      if (serialBufferIndex > 0)
      {
        parseSerialCommand(serialBuffer);
      }

      serialBufferIndex = 0;
    }
    else
    {
      if (serialBufferIndex < (int)(sizeof(serialBuffer) - 1))
      {
        serialBuffer[serialBufferIndex] = c;
        serialBufferIndex++;
      }
      else
      {
        // Falls zu viele Zeichen kommen, Buffer zurücksetzen.
        serialBufferIndex = 0;
      }
    }
  }
}

void parseSerialCommand(char *cmd)
{
  // Format 1: VOLTAGE;20.10
  if (startsWith(cmd, "VOLTAGE;"))
  {
    batteryVoltage_V = atof(cmd + 8);
    return;
  }

  // Format 2: V=20.10
  if (startsWith(cmd, "V="))
  {
    batteryVoltage_V = atof(cmd + 2);
    return;
  }

  // Format 3: SET_VOLTAGE=20.10
  if (startsWith(cmd, "SET_VOLTAGE="))
  {
    batteryVoltage_V = atof(cmd + 12);
    return;
  }
}

bool startsWith(const char *text, const char *prefix)
{
  while (*prefix)
  {
    if (*text != *prefix)
    {
      return false;
    }

    text++;
    prefix++;
  }

  return true;
}

// ------------------------------------------------------------
// AUSGABE
// ------------------------------------------------------------

void printDataLine(unsigned long dtMs)
{
  float voltageCorrection_V = getVoltageTemperatureCorrection();
  float voltageCorrected_V = getTemperatureCorrectedVoltage();
  float voltagePercent = voltageToPercent(voltageCorrected_V);
  float remainingPercent = (remainingAh / TOTAL_CAPACITY_AH) * 100.0;
  float runtimeMin = calculateRuntimeRemainingMin();


  Serial.print(F("DATA;ms="));
  Serial.print(millis());

  Serial.print(F(";dt_ms="));
  Serial.print(dtMs);

  Serial.print(F(";voltage_pin_v="));
  Serial.print(voltagePin_V, 3);

  Serial.print(F(";voltage_v="));
  Serial.print(batteryVoltage_V, 2);

  Serial.print(F(";current_v="));
  Serial.print(currentVoltage_V, 3);

  Serial.print(F(";current_a="));
  Serial.print(current_A, 2);

  Serial.print(F(";current_avg_a="));
  Serial.print(currentAvg_A, 2);

  // Temperaturabhängigkeit der Spannung.
  Serial.print(F(";temp_c="));
  Serial.print(temp_C, 1);
  Serial.print(F(";temp_correction_v="));
  Serial.print(voltageCorrection_V, 3);

  Serial.print(F(";voltage_corrected_v="));
  Serial.print(voltageCorrected_V, 2);

  Serial.print(F(";remaining_ah="));
  Serial.print(remainingAh, 4);

  Serial.print(F(";remaining_percent="));
  Serial.print(remainingPercent, 2);

  // Prozentwert basiert auf temperaturkorrigierter Spannung.
  Serial.print(F(";voltage_percent="));
  Serial.print(voltagePercent, 2);

  Serial.print(F(";runtime_remaining_min="));
  Serial.print(runtimeMin, 1);

  Serial.println();
}

// ------------------------------------------------------------
// HILFSFUNKTIONEN
// ------------------------------------------------------------

float absFloat(float value)
{
  if (value < 0.0)
  {
    return -value;
  }

  return value;
}
