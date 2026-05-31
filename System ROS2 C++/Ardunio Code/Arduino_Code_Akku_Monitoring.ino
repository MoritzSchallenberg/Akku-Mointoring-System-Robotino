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

// ACS758-Ausgang.
// Wichtig: Dieser Pin muss mit OUT des ACS758 verbunden sein.
const int CURRENT_PIN = A1;

// Temperaturleitungen.
// Diese Pins lesen aktuell die drei Temperaturkanäle ein.
const int TEMP_PIN_LEFT   = A3;
const int TEMP_PIN_CENTER = A4;
const int TEMP_PIN_RIGHT  = A5;

// ------------------------------------------------------------
// ALLGEMEINE EINSTELLUNGEN
// ------------------------------------------------------------

// Arduino UNO ADC-Referenz.
// Standardmäßig misst analogRead gegen 5 V.
const float VCC = 5.0;

// 10-bit ADC: Wertebereich 0 bis 1023.
const float ADC_MAX = 1023.0;

// ------------------------------------------------------------
// STROMSENSOR-EINSTELLUNGEN
// ------------------------------------------------------------

// ACS758-200B: ungefähr 10 mV pro Ampere.
// Falls euer Sensor eine andere Variante ist, muss dieser Wert angepasst werden.
const float ACS758_SENSITIVITY_V_PER_A = 0.010;

// Deshalb ist 2.450 V aktuell der beste 0-A-Punkt.
// Dieser Offset wurde aus deinen Messdaten abgeleitet und muss gegebenenfalls abgeleitet werden.
const float CURRENT_ZERO_OFFSET_V = 2.450;

// Richtung der Strommessung.
const float CURRENT_DIRECTION = -1.0;

// Kleine Ströme unterhalb dieser Grenze werden als 0 A behandelt, um Offsetrauschen und ADC-Schwankungen zu unterdrücken.
const float CURRENT_CUTOFF_A = 0.7;

// Wie oft der Strom intern gemessen wird.
// Die DATA-Ausgabe kann langsamer sein; intern wird trotzdem häufiger gemessen.
const unsigned long CURRENT_SAMPLE_INTERVAL_MS = 200;

// Für den Durchschnittsstrom werden die letzten 100 Strommesswerte verwendet.
const int CURRENT_AVG_WINDOW_SIZE = 100;

// ------------------------------------------------------------
// AKKU-EINSTELLUNGEN
// ------------------------------------------------------------

// Das System geht fest von 4 Akkus aus.
const int FIXED_BATTERY_COUNT = 4;

// Jeder Akku wird mit 4 Ah angenommen.
const float CAPACITY_PER_BATTERY_AH = 4.0;

// Daraus folgt Gesamtkapazität.
const float FIXED_TOTAL_CAPACITY_AH =
  FIXED_BATTERY_COUNT * CAPACITY_PER_BATTERY_AH;

// ------------------------------------------------------------
// TEMPERATUR-EINSTELLUNGEN
// ------------------------------------------------------------

// Der Arduino nutzt den internen Pullup als Spannungsteiler.
const bool USE_INTERNAL_PULLUP_FOR_TEMP = true;

// Grob angenommener Pullup-Widerstand.
const float TEMP_PULLUP_OHM = 3300.0;

// Standardannahme: 10k-NTC bei 25 °C.
const float NTC_R0_OHM = 10000.0;
const float NTC_T0_K = 298.15; 
const float NTC_BETA = 3950.0;

// Wenn die Temperaturmessung ungültig ist:
const float TEMP_FALLBACK_C = 25.0;

// Spannungswerte Temperatursensor
const float TEMP_MIN_VALID_V = 0.10;
const float TEMP_MAX_VALID_V = 3.70;

// ------------------------------------------------------------
// AUSGABE- UND MITTELUNGSINTERVALLE
// ------------------------------------------------------------

// Alle 1000 ms wird eine komplette DATA-Zeile ausgegeben.
const unsigned long DATA_INTERVAL_MS = 1000;

// Anzahl ADC-Samples pro Messwert.
const int CURRENT_ADC_SAMPLES = 20;
const int TEMP_ADC_SAMPLES = 20;

// ------------------------------------------------------------
// LAUFZEITVARIABLEN
// ------------------------------------------------------------

unsigned long lastCurrentSampleMs = 0;
unsigned long lastDataMs = 0;

// Aktueller Stromsensor-Spannungswert
float currentVoltage_V = 0.0;

// Aktueller Momentanstrom
float current_A = 0.0;

// Ringpuffer für den gleitenden Durchschnittsstrom.
// Es werden immer die letzten CURRENT_AVG_WINDOW_SIZE Messwerte verwendet.
float currentAvgBuffer_A[CURRENT_AVG_WINDOW_SIZE];
int currentAvgIndex = 0;
int currentAvgCount = 0;

// Temperaturwerte
float tempLeft_V = 0.0;
float tempCenter_V = 0.0;
float tempRight_V = 0.0;

float tempLeft_C = TEMP_FALLBACK_C;
float tempCenter_C = TEMP_FALLBACK_C;
float tempRight_C = TEMP_FALLBACK_C;

float tempAvg_C = TEMP_FALLBACK_C;
int tempValidCount = 0;

// ------------------------------------------------------------
// SETUP
// ------------------------------------------------------------

void setup()
{
  Serial.begin(115200);

  // Stromsensor-Pin als Analoginput.
  pinMode(CURRENT_PIN, INPUT);

  // Temperaturpins vorbereiten.
  // INPUT_PULLUP aktiviert den internen Pullup-Widerstand.
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

  // Ringpuffer für den Durchschnittsstrom initialisieren.
  for (int i = 0; i < CURRENT_AVG_WINDOW_SIZE; i++)
  {
    currentAvgBuffer_A[i] = 0.0;
  }

  // Erste Messungen durchführen, damit keine leeren Werte gesendet werden.
  readCurrentSensor();
  addCurrentToAverageBuffer();
  readTemperatures();

  lastCurrentSampleMs = millis();
  lastDataMs = millis();

  // Startinformationen
  Serial.print(F(";currentZeroOffsetV="));
  Serial.print(CURRENT_ZERO_OFFSET_V, 3);
  Serial.print(F(";totalCapacityAh="));
  Serial.println(FIXED_TOTAL_CAPACITY_AH, 2);
}

// ------------------------------------------------------------
// LOOP
// ------------------------------------------------------------

void loop()
{
  unsigned long now = millis();

  // 1) Strom regelmäßig intern messen.
  if (now - lastCurrentSampleMs >= CURRENT_SAMPLE_INTERVAL_MS)
  {
    lastCurrentSampleMs = now;
    readCurrentSensor();
    addCurrentToAverageBuffer();
  }

  // 2) Einmal pro Sekunde eine DATA-Zeile senden.
  if (now - lastDataMs >= DATA_INTERVAL_MS)
  {
    unsigned long dtMs = now - lastDataMs;
    lastDataMs = now;
    readTemperatures();

    // Durchschnitt über die letzten 100 Strommesswerte berechnen.
    float currentAvg_A = getAverageCurrent();

    printDataLine(dtMs, currentAvg_A);
  }
}

// ------------------------------------------------------------
// ANALOGE HILFSFUNKTIONEN
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
  // Spannung am ACS758-Ausgang messen.
  currentVoltage_V = readVoltageAverage(CURRENT_PIN, CURRENT_ADC_SAMPLES);

  // Spannung in Ampere umrechnen.
  current_A =
    ((currentVoltage_V - CURRENT_ZERO_OFFSET_V) / ACS758_SENSITIVITY_V_PER_A)
    * CURRENT_DIRECTION;

  // Kleines Rauschen um 0 A entfernen.
  if (absFloat(current_A) < CURRENT_CUTOFF_A)
  {
    current_A = 0.0;
  }
}

void addCurrentToAverageBuffer()
{
  // Für Verbrauch interessiert uns der Betrag des Stroms.
  // Dadurch wird ein negativer Messwert nicht als negativer Verbrauch gezählt.
  currentAvgBuffer_A[currentAvgIndex] = absFloat(current_A);

  currentAvgIndex++;

  if (currentAvgIndex >= CURRENT_AVG_WINDOW_SIZE)
  {
    currentAvgIndex = 0;
  }

  if (currentAvgCount < CURRENT_AVG_WINDOW_SIZE)
  {
    currentAvgCount++;
  }
}

float getAverageCurrent()
{
  if (currentAvgCount == 0)
  {
    return 0.0;
  }

  float sum_A = 0.0;

  for (int i = 0; i < currentAvgCount; i++)
  {
    sum_A += currentAvgBuffer_A[i];
  }

  return sum_A / (float)currentAvgCount;
}

// ------------------------------------------------------------
// TEMPERATURMESSUNG
// ------------------------------------------------------------

float readTemperatureVoltage(int pin)
{
  if (USE_INTERNAL_PULLUP_FOR_TEMP)
  {
    // Pullup sicher aktivieren.
    pinMode(pin, INPUT_PULLUP);
    delay(3);
  }

  return readVoltageAverage(pin, TEMP_ADC_SAMPLES);
}

bool voltageLooksLikeValidTemp(float voltage)
{
  return (voltage > TEMP_MIN_VALID_V && voltage < TEMP_MAX_VALID_V);
}

float ntcVoltageToCelsius(float voltage)
{
  // Schutz gegen Division durch 0 und ungültige ADC-Werte.
  if (voltage <= 0.0 || voltage >= VCC)
  {
    return TEMP_FALLBACK_C;
  }

  // Spannungsteiler:
  // 5V -> Pullup -> Analogpin -> NTC -> GND
  //
  // U = VCC * R_ntc / (R_pullup + R_ntc)
  // Umgestellt:
  // R_ntc = R_pullup * U / (VCC - U)
  float ntcResistance =
    TEMP_PULLUP_OHM * voltage / (VCC - voltage);

  // Beta-Gleichung:
  // 1/T = 1/T0 + (1/B) * ln(R/R0)
  float tempK =
    1.0 / ((1.0 / NTC_T0_K) +
           (1.0 / NTC_BETA) *
           log(ntcResistance / NTC_R0_OHM));
  float tempC = tempK - 273.15;
  return tempC;
}

void readTemperatures()
{
  tempLeft_V = readTemperatureVoltage(TEMP_PIN_LEFT);
  tempCenter_V = readTemperatureVoltage(TEMP_PIN_CENTER);
  tempRight_V = readTemperatureVoltage(TEMP_PIN_RIGHT);
  tempValidCount = 0;
  float tempSum_C = 0.0;

  if (voltageLooksLikeValidTemp(tempLeft_V))
  {
    tempLeft_C = ntcVoltageToCelsius(tempLeft_V);
    tempSum_C += tempLeft_C;
    tempValidCount++;
  }
  else
  {
    tempLeft_C = TEMP_FALLBACK_C;
  }

  if (voltageLooksLikeValidTemp(tempCenter_V))
  {
    tempCenter_C = ntcVoltageToCelsius(tempCenter_V);
    tempSum_C += tempCenter_C;
    tempValidCount++;
  }
  else
  {
    tempCenter_C = TEMP_FALLBACK_C;
  }

  if (voltageLooksLikeValidTemp(tempRight_V))
  {
    tempRight_C = ntcVoltageToCelsius(tempRight_V);
    tempSum_C += tempRight_C;
    tempValidCount++;
  }
  else
  {
    tempRight_C = TEMP_FALLBACK_C;
  }

  if (tempValidCount > 0)
  {
    tempAvg_C = tempSum_C / (float)tempValidCount;
  }
  else
  {
    tempAvg_C = TEMP_FALLBACK_C;
  }
}

// ------------------------------------------------------------
// AUSGABE
// ------------------------------------------------------------

void printDataLine(unsigned long dtMs, float currentAvg_A)
{
  Serial.print(F(";dt_ms="));
  Serial.print(dtMs);

  Serial.print(F(";current_a="));
  Serial.print(current_A, 2);

  Serial.print(F(";current_avg_a="));
  Serial.print(currentAvg_A, 2);

  Serial.print(F(";temp_avg_c="));
  Serial.print(tempAvg_C, 1);

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
