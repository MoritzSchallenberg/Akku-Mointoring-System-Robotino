# 🔋 Akku Monitoring System – Robotino
Ein System zur Überwachung und Analyse von Akkuzuständen für den Festo Robotino.  
Ziel ist die präzise Erfassung von Strom, Spannung und Zustandsdaten mehrerer Akkus zur Optimierung von Laufzeit, Sicherheit und Systemverhalten.


## 🚀 Projektübersicht
Dieses Projekt entwickelt ein **Akkumonitoring-System für Robotino**, das folgende Funktionen ermöglicht:
- Messung von Strom und Spannung
- Überwachung mehrerer Akkus gleichzeitig
- Berechnung des Ladezustands (State of Charge)
- Erkennung von Ein-/Aussteckzuständen
- Temperaturüberwachung
- Integration in ROS / Robotino-Systeme

Das System dient insbesondere für Anwendungen in:
- 🤖 Robotik (z. B. RoboCup / Smart Manufacturing)
- 🔋 Energiemanagement
- ⚙️ industrielle Automatisierung


## 🧠 Funktionsprinzip
Das Monitoring basiert auf einer Kombination aus:
- Strommessung (z. B. ACS758 Sensor)
- Spannungsmessung
- Coulomb Counting
- Zusatzsignalen pro Akku:
  - Temperatur
  - Maschinen-ID
  - Plug-In / Plug-Out Status

Die Daten werden kontinuierlich erfasst und verarbeitet.


## ⚙️ Systemarchitektur
### Komponenten:
- 🔋 Akkus (z. B. 18V Systeme, parallel geschaltet)
- 📟 Stromsensor (ACS758-200B)
- 🔌 Analogsignale (Temp, ID, Status)
- 💻 Rechner (z. B. Intel NUC / PC)
- 🤖 Robotino Plattform
- 🧠 ROS (Datenverarbeitung)

## 📊 Features
### ✅ Aktuell umgesetzt
- Grundlegende Messung von Strom und Spannung
- Integration von Sensorsignalen
- Datenaufnahme über analoge Eingänge
- erste Auswertung der Akkudaten

### 🔄 In Entwicklung
- präzise SOC-Berechnung (Coulomb Counting)
- Kalman-Filter zur Glättung der Messwerte
- bessere Fehlererkennung (Spannung über... = Fehler Messung wird nicht mit eingebaut)
- Visualisierung der Daten
- automatische Akku-Erkennung
- Vorhersage der Restlaufzeit mit Parameter durchschnittlicher Verbrauch

## 🛠️ Technologien
- **Hardware:**
  - Festo Robotino
  - ACS758 Stromsensor
  - Mikrocontroller (z. B. Arduino / ESP / direkte PC-Anbindung)

- **Software:**
  - ROS / ROS2
  - Python / C++
  - Node-basierte Kommunikation

Robotino stellt bereits Sensordaten über ROS-Topics bereit (z. B. Motor- und Power-Daten), diese sind jedoch teilweise unzuverlässig oder unvollständig :contentReference[oaicite:0]{index=0}.

## 📂 Projektstruktur
