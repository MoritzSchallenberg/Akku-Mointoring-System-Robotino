/***************************************************************************
 *  battery_state_estimator.cpp - ROS2 Read in Ardunio Akku Monitoring
 *
 *  Created: May 18 12:00:00 2026
 *  Copyright  2026-2029  Moritz Schallenberg

 *  Sie liest zwei kompakte State-Topics ein:
 *    1) /battery/arduino_state
 *       Kommt aus der Arduino-ReadIn-Node.
 *       Enthält z. B.:
 *       ARDUINO_BATTERY_STATE;ros_stamp_ns=...;arduino_ms=...;dt_ms=1000;current_a=2.3;current_avg_a=2.1;temp_avg_c=28.4;battery_count=4;capacity_ah=16.0;battery_mode=fixed_4_batteries
 *
 *    2) /battery/robotino_voltage_state
 *       Kommt aus der Robotino-Spannungs-Node.
 *       Enthält z. B.:
 *       ROBOTINO_VOLTAGE_STATE;ros_stamp_ns=...;idle_valid=1;idle_time_s=21.5;voltage_v=20.1;voltage_percent=98.0

 *  Daraus berechnet diese Node:
 *    - Durchschnittsstrom aus dem Arduino-State
 *    - Verbrauch im letzten Berechnungsintervall in Ah
 *    - Restladung in Ah
 *    - Restladung in Prozent
 *    - Restlaufzeit in Minuten
 *    - Spannungskorrektur der Restladung, aber nur wenn idle_valid=1
 *    - Warnstatus

 *  Danach published sie nur noch EINEN zusammengefassten State:
 *    /battery/state
 *    einfach mit `ros2 topic echo /battery/state` lesbar ist.
 
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

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <map>
#include <sstream>
#include <string>

class BatteryStateEstimator : public rclcpp::Node
{
public:
  BatteryStateEstimator()
  : Node("battery_state_estimator")
  {
    // ------------------------------------------------------------
    // Parameter für die Spannungs-/Restladungsrechnung
    // ------------------------------------------------------------
    // Untere Spannung, bei der wir den Akku als praktisch leer ansehen.
    u_min_v_ = this->declare_parameter<double>("u_min_v", 17.2);

    // Spannung eines vollen 18-V-Akkusystems unter realen Bedingungen.
    u_max_v_ = this->declare_parameter<double>("u_max_v", 20.5);

    // Sicherheitsabschlag auf die Spannung, damit die Spannungsschätzung nicht zu optimistisch ist.
    u_error_v_ = this->declare_parameter<double>("u_error_v", 0.2);

    // Sicherheitsfaktor für die nutzbare Kapazität.
    // 1.0 = volle gemeldete Kapazität nutzen.
    // 0.85 = nur 85 % als nutzbar annehmen.
    k_capacity_safety_ = this->declare_parameter<double>("k_capacity_safety", 1.0);

    // Gewichtung zwischen Stromintegration und Spannungskorrektur.
    // 0.8 bedeutet:
    //   80 % bisherige Coulomb-Counting-Schätzung
    //   20 % Spannungsschätzung im Stillstand
    alpha_coulomb_counting_ = this->declare_parameter<double>("alpha_coulomb_counting", 0.8);

    // Unterhalb dieses Durchschnittsstroms berechnen wir keine sinnvolle Restlaufzeit, weil die Rechnung bei sehr kleinen Strömen unrealistisch große Werte ergibt.
    runtime_min_current_a_ = this->declare_parameter<double>("runtime_min_current_a", 0.5);

    // Wie oft diese Node neu rechnet und /battery/state published.
    update_period_s_ = this->declare_parameter<double>("update_period_s", 10.0);

    // Optionaler Temperaturausgleich für die Spannung.
    // Aktuell bleibt k_u_t_v_per_c standardmäßig 0.0, weil eure Temperaturumrechnung noch nur eine grobe NTC-Schätzung ist.
    temp_ref_c_ = this->declare_parameter<double>("temp_ref_c", 25.0);
    k_u_t_v_per_c_ = this->declare_parameter<double>("k_u_t_v_per_c", 0.0);

    // Wenn true, wartet die Node beim Start auf eine gültige Stillstandsspannung und initialisiert daraus die Restladung. 
    // Wenn keine gültige Spannung kommt, startet sie erstmal mit voller Kapazität.
    initialize_from_voltage_when_available_ =
      this->declare_parameter<bool>("initialize_from_voltage_when_available", true);

    // ------------------------------------------------------------
    // Subscriber: kompakte States aus den beiden vorherigen Nodes
    // ------------------------------------------------------------

    // Arduino-State enthält Strom, Durchschnittsstrom, Temperatur und Kapazität.
    arduino_state_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/battery/arduino_state", 20,
      [this](const std_msgs::msg::String::SharedPtr msg)
      {
        parseArduinoState(msg->data);
      });

    // Robotino-Voltage-State enthält Spannung und ob diese Spannung im Stillstand gültig ist.
    robotino_voltage_state_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/battery/robotino_voltage_state", 20,
      [this](const std_msgs::msg::String::SharedPtr msg)
      {
        parseRobotinoVoltageState(msg->data);
      });

    // ------------------------------------------------------------
    // Publisher: nur noch ein finaler Gesamt-State
    // ------------------------------------------------------------
    battery_state_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/battery/state", 10);

    last_update_time_ = this->now();

    // Timer ruft regelmäßig updateCalculation() auf.
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(update_period_s_ * 1000.0)),
      std::bind(&BatteryStateEstimator::updateCalculation, this));

    RCLCPP_INFO(this->get_logger(), "battery_state_estimator started.");
    RCLCPP_INFO(this->get_logger(), "Subscribing: /battery/arduino_state and /battery/robotino_voltage_state");
    RCLCPP_INFO(this->get_logger(), "Publishing:  /battery/state");
  }

private:
  // ------------------------------------------------------------
  // Hilfsfunktionen für Parsing und Zahlenumwandlung
  // ------------------------------------------------------------

  static double clamp(double value, double min_value, double max_value)
  {
    return std::max(min_value, std::min(value, max_value));
  }

  // Der State wird als Text mit Semikolon getrennt übertragen:
  //   PREFIX;key=value;key=value;key=value
  // Diese Funktion zerlegt so einen String in eine Map:
  //   map["current_avg_a"] = "2.1"
  std::map<std::string, std::string> parseKeyValueState(const std::string & line) const
  {
    std::map<std::string, std::string> values;
    std::stringstream ss(line);
    std::string token;

    // Erstes Token ist meistens nur der Prefix, z. B. ARDUINO_BATTERY_STATE.
    // Tokens ohne '=' werden ignoriert.
    while (std::getline(ss, token, ';')) {
      const std::size_t pos = token.find('=');
      if (pos == std::string::npos) {
        continue;
      }

      const std::string key = token.substr(0, pos);
      const std::string value = token.substr(pos + 1);
      values[key] = value;
    }

    return values;
  }

  // Versucht einen double-Wert aus der Map zu lesen.
  // Falls der Wert fehlt oder nicht konvertierbar ist, bleibt der alte Wert erhalten.
  bool getDouble(
    const std::map<std::string, std::string> & values,
    const std::string & key,
    double & target) const
  {
    const auto it = values.find(key);
    if (it == values.end()) {
      return false;
    }

    try {
      target = std::stod(it->second);
      return true;
    } catch (...) {
      return false;
    }
  }

  // Versucht einen int-Wert aus der Map zu lesen.
  bool getInt(
    const std::map<std::string, std::string> & values,
    const std::string & key,
    int & target) const
  {
    const auto it = values.find(key);
    if (it == values.end()) {
      return false;
    }

    try {
      target = std::stoi(it->second);
      return true;
    } catch (...) {
      return false;
    }
  }

  // Versucht einen bool-Wert aus der Map zu lesen.
  // Akzeptiert 1/0, true/false, TRUE/FALSE.
  bool getBool(
    const std::map<std::string, std::string> & values,
    const std::string & key,
    bool & target) const
  {
    const auto it = values.find(key);
    if (it == values.end()) {
      return false;
    }

    const std::string value = it->second;
    if (value == "1" || value == "true" || value == "TRUE") {
      target = true;
      return true;
    }

    if (value == "0" || value == "false" || value == "FALSE") {
      target = false;
      return true;
    }

    return false;
  }

  // ------------------------------------------------------------
  // Eingangs-State 1: Arduino-State auswerten
  // ------------------------------------------------------------
  void parseArduinoState(const std::string & line)
  {
    const auto values = parseKeyValueState(line);

    // current_a ist der Momentanstrom.
    getDouble(values, "current_a", latest_current_a_);

    // current_avg_a kommt bereits vom Arduino.
    getDouble(values, "current_avg_a", latest_average_current_a_);

    // Temperatur kommt bereits als grobe °C-Schätzung vom Arduino.
    getDouble(values, "temp_avg_c", latest_temp_avg_c_);

    // Kapazität kommt aktuell fest vom Arduino: 4 Akkus * 4 Ah = 16 Ah.
    getDouble(values, "capacity_ah", total_capacity_ah_);

    // battery_count wird nur für State/Debug übernommen.
    getInt(values, "battery_count", battery_count_);

    // Zeitwerte vom Arduino sind hilfreich für Debugging.
    getDouble(values, "dt_ms", latest_arduino_dt_ms_);
    getDouble(values, "arduino_ms", latest_arduino_ms_);

    has_arduino_state_ = true;
  }

  // ------------------------------------------------------------
  // Eingangs-State 2: Robotino-Spannungs-State auswerten
  // ------------------------------------------------------------
  void parseRobotinoVoltageState(const std::string & line)
  {
    const auto values = parseKeyValueState(line);

    // Spannung in Volt.
    if (getDouble(values, "voltage_v", latest_voltage_v_)) {
      has_voltage_ = true;
    }

    // Spannung in Prozent bezogen auf Vollspannung. Wird nicht direkt für die Rechnung genutzt, aber im State wieder ausgegeben, weil es für Anzeige/Debug praktisch ist.
    getDouble(values, "voltage_percent", latest_voltage_percent_);

    // idle_valid sagt, ob die Spannung im Stillstand gemessen wurde.
    // Nur dann wird sie zur Restladungs-Korrektur verwendet.
    getBool(values, "idle_valid", voltage_idle_valid_);

    // idle_time_s ist nur Info/Debug.
    getDouble(values, "idle_time_s", latest_idle_time_s_);

    has_voltage_state_ = true;
  }

  // ------------------------------------------------------------
  // Spannung -> geschätzte Restkapazität
  // ------------------------------------------------------------
  double calculateVoltageBasedCapacityAh(double voltage_v, double temperature_c) const
  {
    // Optionale Temperaturkorrektur:
    const double u_ref_v = voltage_v + k_u_t_v_per_c_ * (temp_ref_c_ - temperature_c);

    // Sicherheitsabschlag auf die Spannung.
    const double u_safe_v = u_ref_v - u_error_v_;

    // Lineare Spannungs-SoC-Schätzung:
    // U_min -> 0 %
    // U_max -> 100 %
    double soc_from_voltage = 0.0;
    if (u_max_v_ > u_min_v_) {
      soc_from_voltage = (u_safe_v - u_min_v_) / (u_max_v_ - u_min_v_);
    }

    soc_from_voltage = clamp(soc_from_voltage, 0.0, 1.0);

    // Aus dem Spannungs-SoC wird eine Ah-Schätzung.
    return total_capacity_ah_ * soc_from_voltage * k_capacity_safety_;
  }

  // ------------------------------------------------------------
  // Initialisierung der Restladung
  // ------------------------------------------------------------
  void initializeRemainingAhIfNeeded()
  {
    if (remaining_initialized_) {
      return;
    }

    if (initialize_from_voltage_when_available_ && has_voltage_ && voltage_idle_valid_) {
      remaining_ah_ = calculateVoltageBasedCapacityAh(latest_voltage_v_, latest_temp_avg_c_);
    } else {
      // Fallback: Wenn noch keine gültige Spannung da ist, starten wir konservativ mit der aktuell gemeldeten Kapazität.
      remaining_ah_ = total_capacity_ah_ * k_capacity_safety_;
    }

    remaining_initialized_ = true;
  }

  // ------------------------------------------------------------
  // Warnstatus bestimmen
  // ------------------------------------------------------------
  std::string buildWarning(double remaining_percent, double runtime_min) const
  {
    if (!has_arduino_state_) {
      return "NO_ARDUINO_STATE";
    }

    if (!has_voltage_state_) {
      return "NO_ROBOTINO_VOLTAGE_STATE";
    }

    if (total_capacity_ah_ <= 0.0) {
      return "NO_CAPACITY_DATA";
    }

    if (latest_temp_avg_c_ >= 60.0) {
      return "CRITICAL_TEMPERATURE";
    }

    if (latest_temp_avg_c_ >= 50.0) {
      return "HIGH_TEMPERATURE";
    }

    if (remaining_percent <= 10.0) {
      return "CRITICAL_BATTERY";
    }

    if (remaining_percent <= 20.0) {
      return "LOW_BATTERY";
    }

    if (runtime_min >= 0.0 && runtime_min <= 10.0) {
      return "LOW_RUNTIME";
    }

    return "OK";
  }

  // ------------------------------------------------------------
  // Hauptberechnung
  // ------------------------------------------------------------
  void updateCalculation()
  {
    const rclcpp::Time now = this->now();

    // dt_s ist die Zeit seit der letzten Berechnung.
    // Daraus berechnen wir, wie viel Ah im letzten Intervall verbraucht wurde.
    const double dt_s = std::max(0.0, (now - last_update_time_).seconds());
    last_update_time_ = now;

    // Wenn noch keine Arduino-Daten da sind, macht eine Verbrauchsrechnung keinen Sinn.
    if (!has_arduino_state_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        5000,
        "Waiting for /battery/arduino_state ...");
      return;
    }

    // Restladung beim ersten gültigen Update initialisieren.
    initializeRemainingAhIfNeeded();

    // Der Arduino berechnet current_avg_a bereits selbst.
    const double average_current_a = std::abs(latest_average_current_a_);

    // Ah-Verbrauch des letzten Berechnungsintervalls.
    const double consumed_ah_interval = average_current_a * dt_s / 3600.0;

    // Coulomb Counting: Restladung wird um den verbrauchten Anteil reduziert.
    double remaining_cc_ah = remaining_ah_ - consumed_ah_interval;
    remaining_cc_ah = clamp(remaining_cc_ah, 0.0, total_capacity_ah_);

    // Spannungsschätzung initial auf -1 setzen, damit man im State erkennt,
    double voltage_capacity_ah = -1.0;
    bool voltage_correction_applied = false;

    // Spannungskorrektur wird nur angewendet, wenn die Robotino-Voltage-Node
    if (has_voltage_ && voltage_idle_valid_) {
      voltage_capacity_ah = calculateVoltageBasedCapacityAh(latest_voltage_v_, latest_temp_avg_c_);

      // Mischung aus Coulomb Counting und Spannungsschätzung.
      remaining_ah_ =
        alpha_coulomb_counting_ * remaining_cc_ah +
        (1.0 - alpha_coulomb_counting_) * voltage_capacity_ah;

      voltage_correction_applied = true;
    } else {
      remaining_ah_ = remaining_cc_ah;
    }

    remaining_ah_ = clamp(remaining_ah_, 0.0, total_capacity_ah_);

    double remaining_percent = 0.0;
    if (total_capacity_ah_ > 0.0) {
      remaining_percent = 100.0 * remaining_ah_ / total_capacity_ah_;
    }

    // Restlaufzeit nur berechnen, wenn der Durchschnittsstrom sinnvoll groß ist.
    double runtime_remaining_min = -1.0;
    if (average_current_a >= runtime_min_current_a_) {
      runtime_remaining_min = (remaining_ah_ / average_current_a) * 60.0;
    }

    const std::string warning = buildWarning(remaining_percent, runtime_remaining_min);

    // ------------------------------------------------------------
    // Finalen State bauen
    // ------------------------------------------------------------
    std::ostringstream state;
    state << std::fixed << std::setprecision(3)
          << "BATTERY_STATE"
          << ";ros_stamp_ns=" << now.nanoseconds()
          << ";dt_s=" << dt_s
          << ";current_a=" << latest_current_a_
          << ";current_avg_a=" << average_current_a
          << ";consumed_ah_interval=" << consumed_ah_interval
          << ";remaining_ah=" << remaining_ah_
          << ";remaining_percent=" << remaining_percent
          << ";runtime_remaining_min=" << runtime_remaining_min
          << ";capacity_ah=" << total_capacity_ah_
          << ";battery_count=" << battery_count_
          << ";temp_avg_c=" << latest_temp_avg_c_
          << ";voltage_v=" << (has_voltage_ ? latest_voltage_v_ : -1.0)
          << ";voltage_percent=" << latest_voltage_percent_
          << ";voltage_idle_valid=" << (voltage_idle_valid_ ? 1 : 0)
          << ";idle_time_s=" << latest_idle_time_s_
          << ";voltage_capacity_ah=" << voltage_capacity_ah
          << ";voltage_correction_applied=" << (voltage_correction_applied ? 1 : 0)
          << ";warning=" << warning;

    std_msgs::msg::String msg;
    msg.data = state.str();
    battery_state_pub_->publish(msg);

    RCLCPP_INFO(this->get_logger(), "%s", msg.data.c_str());
  }

  // ------------------------------------------------------------
  // Parameter
  // ------------------------------------------------------------
  double u_min_v_ = 17.2;
  double u_max_v_ = 20.5;
  double u_error_v_ = 0.2;
  double k_capacity_safety_ = 1.0;
  double alpha_coulomb_counting_ = 0.8;
  double runtime_min_current_a_ = 0.5;
  double update_period_s_ = 10.0;
  double temp_ref_c_ = 25.0;
  double k_u_t_v_per_c_ = 0.0;
  bool initialize_from_voltage_when_available_ = true;

  // ------------------------------------------------------------
  // Eingangswerte aus /battery/arduino_state
  // ------------------------------------------------------------
  bool has_arduino_state_ = false;
  double latest_current_a_ = 0.0;
  double latest_average_current_a_ = 0.0;
  double latest_temp_avg_c_ = 25.0;
  double total_capacity_ah_ = 16.0;
  int battery_count_ = 4;
  double latest_arduino_dt_ms_ = 0.0;
  double latest_arduino_ms_ = 0.0;

  // ------------------------------------------------------------
  // Eingangswerte aus /battery/robotino_voltage_state
  // ------------------------------------------------------------
  bool has_voltage_state_ = false;
  bool has_voltage_ = false;
  double latest_voltage_v_ = 0.0;
  double latest_voltage_percent_ = -1.0;
  bool voltage_idle_valid_ = false;
  double latest_idle_time_s_ = 0.0;

  // ------------------------------------------------------------
  // interner Zustand der Berechnung
  // ------------------------------------------------------------
  bool remaining_initialized_ = false;
  double remaining_ah_ = 0.0;
  rclcpp::Time last_update_time_;

  // ------------------------------------------------------------
  // ROS Handles
  // ------------------------------------------------------------
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr arduino_state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robotino_voltage_state_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr battery_state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BatteryStateEstimator>());
  rclcpp::shutdown();
  return 0;
}
