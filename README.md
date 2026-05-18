# Akku Monitoring System – Robotino

Dieses Projekt entwickelt ein Akku-Monitoring-System für den Festo Robotino.  
Ziel ist es, Strom, Temperatur, Spannung, Restladung und geschätzte Restlaufzeit der Akkus zu erfassen und über ROS2 bereitzustellen.

Das System kombiniert Messwerte eines Arduino-basierten Sensorsystems mit der internen Spannungsmessung des Robotino. Daraus wird ein gemeinsamer Akku-Zustand berechnet, der von anderen ROS2-Nodes weiterverwendet werden kann.

---

## Ziel des Projekts

Der Robotino soll zusätzliche Informationen über seinen Energiezustand erhalten:

- aktueller Stromverbrauch
- durchschnittlicher Stromverbrauch
- Temperatur der Akkus
- gemessene Robotino-Spannung
- geschätzte Restladung in Ah
- geschätzte Restladung in %
- geschätzte Restlaufzeit
- Warnmeldungen bei kritischen Zuständen

Das Projekt dient als Grundlage für ein robusteres Energiemanagement bei mobilen Robotiksystemen.

---

## Systemübersicht

```text
Arduino
│
├── Strommessung über ACS758
├── Temperaturmessung über Akkusensoren
├── Berechnung von Durchschnittsstrom
└── Ausgabe als Serial-Daten
        ↓
ROS2 Arduino Reader Node
        ↓
/battery/arduino_state
        ↓
Robotino Voltage Reader Node
        ↓
/battery/robotino_voltage_state
        ↓
Battery State Estimator
        ↓
/battery/state
