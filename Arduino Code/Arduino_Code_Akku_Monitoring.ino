
/***************************************************************************
 *  Arduino_Code_Akku_Monitoring.ino - Arduino Akku Monitoring
 *
 *  Created: May 18 12:00:00 2026
 *  Copyright  2026-2029  Moritz Schallenberg

 *  Version:
 *  - ein Strompfad über ACS758 an A1
 *  - feste Akkuzahl: 4 Akkus
 *  - feste Gesamtkapazität: 16 Ah
 *  - Temperatur wird direkt auf dem Arduino grob in °C umgerechnet
 *  - es wird nur die Durchschnittstemperatur ausgegeben
 *  - Durchschnittsstrom wird auf dem Arduino berechnet und ausgegeben
 *  - kein Plug-In/Out mehr
 *
 *  Ausgabeformat:
 *  DATA;ms=...;dt_ms=...;current_a=...;current_avg_a=...;temp_avg_c=...;battery_count=4;capacity_ah=16.00;battery_mode=fixed_4_batteries
 
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

// ------------------------------------------------------------
// PINBELEGUNG
// ------------------------------------------------------------

const int CURRENT_PIN = A1;

const int TEMP_PIN_LEFT   = A3;
const int TEMP_PIN_CENTER = A4;
const int TEMP_PIN_RIGHT  = A5;

// ------------------------------------------------------------
// EINSTELLUNGEN
// ------------------------------------------------------------

const float VCC = 5.0;
const float ADC_MAX = 1023.0;

// ACS758ECB-200B: ca. 10 mV/A = 0.010 V/A.
// Falls ihr einen anderen ACS758-Typ nutzt, muss dieser Wert angepasst werden.
const float ACS758_SENSITIVITY_V_PER_A = 0.010;

// Stromsensor-Nullpunkt.
// Keine Auto-Kalibrierung, weil beim Start bereits der Rechner/Robotino läuft.
// Diesen Wert später einmal möglichst genau bei 0 A messen und hier eintragen.
const float CURRENT_ZERO_OFFSET_V = 2.500;

// Falls der Strom falsch herum angezeigt wird, auf -1.0 setzen.
const float CURRENT_DIRECTION = 1.0;

// Kleine Werte um den Nullpunkt werden als 0 A behandelt.
const float CURRENT_CUTOFF_A = 0.5;

// Feste Akkudaten: 4 Akkus mit je 4 Ah.
const int FIXED_BATTERY_COUNT = 4;
const float CAPACITY_PER_BATTERY_AH = 4.0;
const float FIXED_TOTAL_CAPACITY_AH = FIXED_BATTERY_COUNT * CAPACITY_PER_BATTERY_AH;

// Temperatur:
// Nach den bisherigen Tests sind die Temperaturleitungen wahrscheinlich passive NTCs gegen GND.
// Deshalb wird der interne Pullup des Arduino als einfacher Spannungsteiler genutzt.
const bool USE_INTERNAL_PULLUP_FOR_TEMP = true;

// Grobe NTC-Parameter.
// WICHTIG: Das sind Schätzwerte. Sie sind gut genug für Plausibilität/Trend,
// aber nicht für exakte Temperaturmessung.
const float INTERNAL_PULLUP_OHM = 30000.0;  // Arduino UNO intern grob 20k-50k, hier geschätzt 30k
const float NTC_R0_OHM = 10000.0;           // typischer 10k NTC bei 25°C
const float NTC_BETA = 3950.0;              // typischer Beta-Wert
const float NTC_T0_K = 298.15;              // 25°C in Kelvin

// Falls die Temperaturberechnung unrealistische Werte liefert, kann man hier einen Offset korrigieren.
const float TEMP_OFFSET_C = 0.0;

// Sicherheitsgrenzen für die Ausgabe, damit defekte/offene Sensoren nicht völlig absurde Werte liefern.
const float TEMP_MIN_VALID_C = -20.0;
const float TEMP_MAX_VALID_C = 100.0;
const float TEMP_FALLBACK_C = 25.0;

// Messintervalle
const unsigned long CURRENT_INTERVAL_MS = 500;
const unsigned long DATA_INTERVAL_MS = 1000;

// Mittelwerte
const int CURRENT_SAMPLES = 20;
const int TEMP_SAMPLES = 20;

// ------------------------------------------------------------
// VARIABLEN
// ------------------------------------------------------------

unsigned long lastCurrentMs = 0;
unsigned long lastDataMs = 0;

float currentVoltage_V = 0.0;
float current_A = 0.0;

float currentSum_A = 0.0;
unsigned int currentSampleCount = 0;

float tempLeft_C = TEMP_FALLBACK_C;
float tempCenter_C = TEMP_FALLBACK_C;
float tempRight_C = TEMP_FALLBACK_C;
float tempAverage_C = TEMP_FALLBACK_C;

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

  delay(1000);

  readCurrentSensor();
  readTemperatures();

  lastCurrentMs = millis();
  lastDataMs = millis();

  Serial.println(F("START;AkkuMonitoring=ready"));
  Serial.print(F("CONFIG;currentPin=A1;offsetV="));
  Serial.print(CURRENT_ZERO_OFFSET_V, 3);
  Serial.print(F(";sensitivityVperA="));
  Serial.print(ACS758_SENSITIVITY_V_PER_A, 3);
  Serial.print(F(";fixedBatteryCount="));
  Serial.print(FIXED_BATTERY_COUNT);
  Serial.print(F(";capacityPerBatteryAh="));
  Serial.print(CAPACITY_PER_BATTERY_AH, 2);
  Serial.print(F(";totalCapacityAh="));
  Serial.print(FIXED_TOTAL_CAPACITY_AH, 2);
  Serial.print(F(";tempMode="));
  Serial.println(F("ntc_internal_pullup_estimated"));

  printDataLine(0);
}

// ------------------------------------------------------------
// LOOP
// ------------------------------------------------------------

void loop()
{
  unsigned long now = millis();

  // Strom regelmäßig messen und für Durchschnitt sammeln
  if (now - lastCurrentMs >= CURRENT_INTERVAL_MS)
  {
    lastCurrentMs = now;

    readCurrentSensor();

    currentSum_A += absFloat(current_A);
    currentSampleCount++;
  }

  // Vollständige Daten für den PC ausgeben
  if (now - lastDataMs >= DATA_INTERVAL_MS)
  {
    unsigned long dataDtMs = now - lastDataMs;
    lastDataMs = now;

    readTemperatures();

    float currentAverage_A = getCurrentAverageAndReset();

    printDataLine(dataDtMs, currentAverage_A);
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

float getCurrentAverageAndReset()
{
  float average_A = current_A;

  if (currentSampleCount > 0)
  {
    average_A = currentSum_A / (float)currentSampleCount;
  }

  currentSum_A = 0.0;
  currentSampleCount = 0;

  return average_A;
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

float voltageToTemperatureC(float voltage_V)
{
  // Schutz gegen offene/kurzgeschlossene Sensoren oder ungültige Messwerte.
  if (voltage_V <= 0.02 || voltage_V >= (VCC - 0.02))
  {
    return TEMP_FALLBACK_C;
  }

  // Spannungsteiler: 5V -> interner Pullup -> Analogpin -> NTC -> GND
  // U = VCC * R_NTC / (R_PULLUP + R_NTC)
  // R_NTC = R_PULLUP * U / (VCC - U)
  float ntcResistance_Ohm = INTERNAL_PULLUP_OHM * voltage_V / (VCC - voltage_V);

  if (ntcResistance_Ohm <= 0.0)
  {
    return TEMP_FALLBACK_C;
  }

  // Beta-Gleichung:
  // 1/T = 1/T0 + (1/B) * ln(R/R0)
  float temp_K = 1.0 / ((1.0 / NTC_T0_K) + (1.0 / NTC_BETA) * log(ntcResistance_Ohm / NTC_R0_OHM));
  float temp_C = temp_K - 273.15 + TEMP_OFFSET_C;

  if (temp_C < TEMP_MIN_VALID_C || temp_C > TEMP_MAX_VALID_C)
  {
    return TEMP_FALLBACK_C;
  }

  return temp_C;
}

void readTemperatures()
{
  float tempLeft_V = readTemperatureVoltage(TEMP_PIN_LEFT);
  float tempCenter_V = readTemperatureVoltage(TEMP_PIN_CENTER);
  float tempRight_V = readTemperatureVoltage(TEMP_PIN_RIGHT);

  tempLeft_C = voltageToTemperatureC(tempLeft_V);
  tempCenter_C = voltageToTemperatureC(tempCenter_V);
  tempRight_C = voltageToTemperatureC(tempRight_V);

  tempAverage_C = (tempLeft_C + tempCenter_C + tempRight_C) / 3.0;
}

// ------------------------------------------------------------
// AKKU-KAPAZITÄT
// ------------------------------------------------------------

int getBatteryCount()
{
  return FIXED_BATTERY_COUNT;
}

float getTotalCapacityAh()
{
  return FIXED_TOTAL_CAPACITY_AH;
}

// ------------------------------------------------------------
// AUSGABE
// ------------------------------------------------------------

void printDataLine(unsigned long dataDtMs)
{
  printDataLine(dataDtMs, current_A);
}

void printDataLine(unsigned long dataDtMs, float currentAverage_A)
{
  Serial.print(F("DATA;ms="));
  Serial.print(millis());

  Serial.print(F(";dt_ms="));
  Serial.print(dataDtMs);

  Serial.print(F(";current_a="));
  Serial.print(current_A, 2);

  Serial.print(F(";current_avg_a="));
  Serial.print(currentAverage_A, 2);

  Serial.print(F(";temp_avg_c="));
  Serial.print(tempAverage_C, 1);

  Serial.print(F(";battery_count="));
  Serial.print(getBatteryCount());

  Serial.print(F(";capacity_ah="));
  Serial.print(getTotalCapacityAh(), 2);

  Serial.print(F(";battery_mode="));
  Serial.print(F("fixed_4_batteries"));

  Serial.println();
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
