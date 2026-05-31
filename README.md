# Akku-Monitoring-System für den Festo Robotino

Dieses Repository enthält ein Akku-Monitoring-System für den Festo Robotino des Uni-Teams **Carologistics**.
Ziel des Projekts ist es, Strom, Spannung, Temperatur, Restladung und geschätzte Restlaufzeit der Akkus zu erfassen und für den Betrieb des Roboters nutzbar zu machen.

Das Projekt entstand im Rahmen einer E-Technik-Projektarbeit an der FH Aachen und soll langfristig die Akkuüberwachung beim Programmieren, Testen und bei Wettbewerben verbessern.

---

## Ziel des Projekts

Der Robotino wird über mehrere parallel geschaltete 18-V-Akkus versorgt. Eine reine Spannungsbetrachtung reicht für eine zuverlässige Einschätzung des Akkuzustands nur begrenzt aus, da die Spannung unter Last einbrechen kann und nicht direkt den tatsächlich entnommenen Energieinhalt beschreibt.

Das Akku-Monitoring-System soll daher folgende Werte bereitstellen:

* aktuellen Stromverbrauch
* durchschnittlichen Stromverbrauch
* Akkuspannung
* Temperatur der Akkus
* Restladung in Ah
* Restladung in %
* geschätzte Restlaufzeit
* Warnstatus bei kritischem Zustand

---

## Systemvarianten

Im Projekt wurden zwei technische Varianten betrachtet und umgesetzt.

### 1. C++-/ROS2-System für den Robotino

Das C++-/ROS2-System ist für den direkten Einsatz auf den Robotino-Systemen des Teams vorgesehen.
Dabei liest ein Arduino Strom- und Temperaturdaten ein und überträgt diese per USB/Serial an den Robotino-Rechner. Zusätzlich wird die interne Spannungsmessung des Robotino softwareseitig ausgelesen. Beide Datenquellen werden anschließend in ROS2 zusammengeführt.

Datenfluss:

```text
Arduino-Daten
    ↓
arduino_battery_reader
    ↓
/battery/arduino_state
    ↓
battery_state_estimator
    ↓
/battery/state
```

Parallel dazu:

```text
Robotino-Spannungsmessung
    ↓
robotino_voltage_reader
    ↓
/battery/robotino_voltage_state
    ↓
battery_state_estimator
    ↓
/battery/state
```

Der finale Battery-State kann von anderen ROS2-Komponenten weiterverwendet werden, zum Beispiel für Anzeigen, Diagnosefunktionen oder spätere Sicherheitsfunktionen wie Notstopps bei zu geringer Spannung.

---

### 2. Arduino-Standalone-System

Das Arduino-System ist eine kompaktere Variante für kleinere oder selbstgebaute Roboterplattformen, zum Beispiel den Canvastino.
Hier werden Strom, Spannung und Temperatur direkt auf dem Arduino eingelesen und verarbeitet.

Die Akkuspannung wird über einen Spannungsteiler auf einen für den Arduino zulässigen Bereich reduziert und anschließend im Code auf die reale Batteriespannung zurückgerechnet. Dadurch kann das System auch unabhängig von der internen Robotino-Spannungsmessung eingesetzt werden.

Typischer Datenfluss:

```text
ACS758-200B Stromsensor
Akkuspannung über Spannungsteiler
Temperaturdaten der Akkus
    ↓
Arduino
    ↓
Serial DATA-Ausgabe
    ↓
externer PC / Anzeige / spätere Weiterverarbeitung
```

---

## Hardware

Das System verwendet je nach Variante folgende Komponenten:

| Komponente      | Funktion                                             |
| --------------- | ---------------------------------------------------- |
| Festo Robotino  | Mobiler Roboter und Zielsystem                       |
| Arduino UNO     | Einlesen und Vorverarbeiten der Messwerte            |
| ACS758-200B     | Hall-Effekt-Stromsensor zur Messung des Gesamtstroms |
| 18-V-Akkus      | Energieversorgung des Robotino                       |
| Spannungsteiler | Reduktion der Akkuspannung auf Arduino-Messbereich   |
| USB-Kabel       | Datenübertragung zwischen Arduino und Robotino       |
| externer PC     | Anzeige und Bedienung per SSH oder Terminal          |

Im Robotino-System sind insgesamt sechs Akkus vorgesehen. Aufgrund von Hardwareproblemen werden aktuell jedoch nur vier Akkus aktiv eingesetzt. Bei einer angenommenen Kapazität von 4 Ah pro Akku ergibt sich eine nutzbare Gesamtkapazität von 16 Ah.

---

## Arduino-Datenformat

Der Arduino gibt die Messwerte als serielle `DATA`-Zeile aus.
Ein Beispiel:

```text
DATA;ms=12345;dt_ms=1000;current_v=2.512;current_a=8.60;current_avg_a=8.93;voltage_v=20.76;temp_avg_c=26.3;battery_count=4;capacity_ah=16.00;battery_mode=fixed_4_batteries
```

Wichtige Felder:

| Feld            | Bedeutung                                     |
| --------------- | --------------------------------------------- |
| `ms`            | Arduino-Zeit in Millisekunden                 |
| `dt_ms`         | Zeit seit letzter Ausgabe                     |
| `current_v`     | Ausgangsspannung des Stromsensors             |
| `current_a`     | Momentanstrom in Ampere                       |
| `current_avg_a` | geglätteter Durchschnittsstrom                |
| `voltage_v`     | gemessene Akkuspannung                        |
| `temp_avg_c`    | durchschnittliche Akkutemperatur              |
| `battery_count` | angenommene Anzahl aktiver Akkus              |
| `capacity_ah`   | angenommene Gesamtkapazität                   |
| `battery_mode`  | aktueller Betriebsmodus der Kapazitätsannahme |

---

## Berechnungsprinzip

Die Restladung wird hauptsächlich über Coulomb Counting berechnet.

```text
verbrauchte_Ah = current_avg_A × dt_s / 3600
```

Die verbleibende Kapazität ergibt sich aus:

```text
remaining_ah = capacity_ah - consumed_ah
```

Der prozentuale Ladezustand wird berechnet mit:

```text
remaining_percent = remaining_ah / capacity_ah × 100
```

Die geschätzte Restlaufzeit ergibt sich näherungsweise aus:

```text
runtime_remaining_min = remaining_ah / current_avg_A × 60
```

Im C++-/ROS2-System kann die interne Robotino-Spannungsmessung zusätzlich zur Plausibilisierung und Korrektur der Restladung verwendet werden. Im Arduino-System wird die Spannung direkt über den Spannungsteiler eingelesen.

---

## Repository-Struktur

```text
Akku-Mointoring-System-Robotino/
│
├── Arduino Code/
│   └── Arduino-Code für Strom-, Temperatur- und Spannungsmessung
│
├── src/
│   ├── arduino_battery_reader.cpp
│   ├── robotino_voltage_reader.cpp
│   └── battery_state_estimator.cpp
│
├── CMakeLists.txt
├── package.xml
├── LICENSE.txt
└── README.md
```

---

## Build des ROS2-Packages

Im ROS2-Workspace:

```bash
cd ~/ros2_ws
colcon build --packages-select akku_monitoring_system
source install/setup.bash
```

Der Paketname muss ggf. an den tatsächlichen Namen in `package.xml` angepasst werden.

---

## Start der ROS2-Nodes

Arduino-Reader starten:

```bash
ros2 run akku_monitoring_system arduino_battery_reader
```

Robotino-Spannungsreader starten:

```bash
ros2 run akku_monitoring_system robotino_voltage_reader
```

Battery-State-Estimator starten:

```bash
ros2 run akku_monitoring_system battery_state_estimator
```

Finalen Battery-State anzeigen:

```bash
ros2 topic echo /battery/state
```

---

## Arduino-Standalone-Test

Wenn das Arduino-System ohne ROS2 getestet werden soll, kann der serielle Output direkt am PC oder Robotino gelesen werden.

Beispiel:

```bash
stty -F /dev/ttyACM0 115200
cat /dev/ttyACM0
```

Falls der Arduino auf einem anderen Port liegt:

```bash
ls /dev/ttyACM*
ls /dev/ttyUSB*
```

---

## SSH-/Terminal-Anzeige

Für Tests kann der Battery-State oder der Arduino-Output per SSH auf einem externen PC angezeigt werden. Dadurch kann eine Akkuanzeige ähnlich wie bei einem Laptop entstehen, die während des Programmierens, Testens oder Wettbewerbsbetriebs sichtbar ist.

Beispielhafte Anzeige:

```text
Aktueller Strom:        8.60 A
Durchschnittsstrom:     8.93 A
Kapazität:              16.00 Ah
Restladung:             13.10 Ah
Restladung:             81.9 %
Restlaufzeit:           88 min
Spannung:               20.76 V
Temperatur:             26.3 °C
Warnstatus:             OK
```

---

## Sensorgenauigkeit

Die Bewertung der Sensorgenauigkeit erfolgt auf Grundlage der Herstellerangaben der verwendeten Komponenten.
Für die Strommessung wird der Hall-Effekt-Stromsensor ACS758-200B eingesetzt. Dieser Sensor stellt ein analoges Spannungssignal bereit, das proportional zum gemessenen Strom ist. Für die 200-A-Variante beträgt die Sensitivität 10 mV/A.

Die Spannungsmessung des Arduino-Systems erfolgt über einen Spannungsteiler. Die Genauigkeit hängt dabei insbesondere von den Widerstandstoleranzen, der Referenzspannung des Arduino und der Auflösung des Analog-Digital-Wandlers ab. Für den vorgesehenen Einsatz als robuste Akkuanzeige und Restlaufzeitschätzung ist diese Genauigkeit ausreichend.

---


## Sicherheitshinweise

Dieses Projekt arbeitet mit parallel geschalteten Akkus und hohen Strömen.
Vor Hardwaretests müssen Leitungsquerschnitte, Steckverbinder, Sicherungen und die maximale Strombelastbarkeit geprüft werden.

Wichtige Punkte:

* Arduino-Analogpins dürfen niemals direkt mit 18 V verbunden werden.
* Die Akkuspannung muss über einen Spannungsteiler oder eine geeignete Schutzschaltung reduziert werden.
* Akku-Minus/System-GND muss mit Arduino-GND verbunden sein.
* Akkus sollten nur mit ähnlichem Ladezustand parallel betrieben werden.
* Rückströme zwischen Akkus sollten durch geeignete Schutzmaßnahmen reduziert werden.
* Sicherungen oder Schutzbeschaltungen je Akkuzweig sind empfehlenswert.

---

## Quellen und Dokumentation

* Allegro MicroSystems: ACS758 Current Sensor Datasheet
  https://www.allegromicro.com/~/media/files/datasheets/acs758-datasheet.ashx

* Arduino Documentation: analogRead()
  https://docs.arduino.cc/language-reference/en/functions/analog-io/analogRead/

* ROS2 Documentation
  https://docs.ros.org/

* Festo Robotino / OpenRobotino Documentation
  https://doc.openrobotino.org/

---

## Lizenz

Dieses Projekt steht unter der GPL-3.0-Lizenz.
