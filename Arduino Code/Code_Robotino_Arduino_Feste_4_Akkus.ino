/***************************************************************************
 *  sensor_thread.cpp - Robotino sensor thread
 *
 *  Created: May 18 12:00:00 2026
 *  Copyright  2026-2029  Moritz Schallenberg
 
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

// Das Reduzieren von Offset-Rauschen um den Nullpunkt herum.
const float CURRENT_CUTOFF_A = 0.5;

// Feste Akkudaten:
// Plug-In/Out wird nicht mehr ausgewertet.
// Das System geht immer von 4 eingesteckten Akkus mit je 4 Ah aus.
const int FIXED_BATTERY_COUNT = 4;
const float CAPACITY_PER_BATTERY_AH = 4.0;
const float FIXED_TOTAL_CAPACITY_AH =
  FIXED_BATTERY_COUNT * CAPACITY_PER_BATTERY_AH;

// Temperatur:
// Nach unserer Analyse sind die Temperaturleitungen sehr wahrscheinlich passive
// Temperaturfühler gegen GND. Deshalb nutzen wir den internen Pullup als einfachen Spannungsteiler.
const bool USE_INTERNAL_PULLUP_FOR_TEMP = true;

// Messintervalle
const unsigned long CURRENT_INTERVAL_MS = 500;

// Vollständige DATA-Ausgabe an den PC.
// Der PC kann aus current_a und dt_ms Average, Ah-Verbrauch und SOC berechnen.
const unsigned long DATA_INTERVAL_MS = 1000;

// Temperatur-Averaging
const int TEMP_SAMPLES = 20;

// Strom-Averaging
const int CURRENT_SAMPLES = 20;

// ------------------------------------------------------------
// VARIABLEN
// ------------------------------------------------------------

unsigned long lastCurrentMs = 0;
unsigned long lastDataMs = 0;

unsigned long lastCurrentDtMs = 0;

float currentVoltage_V = 0.0;
float current_A = 0.0;

float tempLeft_V = 0.0;
float tempCenter_V = 0.0;
float tempRight_V = 0.0;

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

  readTemperatures();
  readCurrentSensor();

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
  Serial.println(FIXED_TOTAL_CAPACITY_AH, 2);

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
