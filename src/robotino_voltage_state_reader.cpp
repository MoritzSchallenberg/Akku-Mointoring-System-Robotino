/***************************************************************************
 *  robotino_voltage_state_reader.cpp - Spannungsmessung Robotino
 *
 *  Created: May 18 12:00:00 2026
 *  Copyright  2026-2029  Moritz Schallenberg

* Diese Node subscribed auf den Arduino-State:
 *   /battery/arduino_state
 *
 * Daraus liest sie den aktuellen Strom:
 *   current_avg_a
 * oder falls nicht vorhanden:
 *   current_a
 *
 * Die Robotino-Spannung wird nur dann wirklich verwendet, wenn der Strom
 * länger als idle_required_s unter idle_current_threshold_a liegt.
 *
 * Dadurch wird die Spannung nur im Stillstand bzw. bei sehr kleiner Last
 * gemessen. Das ist wichtig, weil die Batteriespannung unter Last einbricht
 * und dann schlechter für eine SOC-Korrektur geeignet ist.
 *
 * Diese ROS2-Node liest die interne Robotino-Spannung aus und veröffentlicht die Information als EINEN gemeinsamen State-String.
 *   /battery/robotino_voltage_state
 *
 * Beispielausgabe:
 *   ROBOTINO_VOLTAGE_STATE;voltage_v=20.12;voltage_percent=98.15;idle_valid=1;idle_time_s=21.4
 
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


#include <algorithm>   // std::max, std::min
#include <chrono>      // Zeitangaben
#include <cmath>       // std::fabs
#include <memory>      // std::make_shared
#include <regex>       // JSON-Werte aus Robotino-Antwort suchen
#include <sstream>     // String zusammensetzen
#include <stdexcept>   // Exceptions
#include <string>      // std::string
#include <unordered_map> // Key-Value Parsing vom State-String
#include <vector>      // Liste möglicher Spannungs-Keys

#include <curl/curl.h> // HTTP GET zur Robotino-REST-Schnittstelle

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class RobotinoVoltageStateReader : public rclcpp::Node
{
public:
  RobotinoVoltageStateReader()
  : Node("robotino_voltage_state_reader")
  {
    // ------------------------------------------------------------
    // Parameter
    // ------------------------------------------------------------
    // robotino_url:
    //   REST-Adresse, über die die Robotino-Powermanagementdaten gelesen werden.
    //   Je nach Robotino/Netzwerk musst du diese URL später anpassen.
    this->declare_parameter<std::string>(
      "robotino_url",
      "http://127.0.0.1/data/powermanagement");

    // arduino_state_topic:
    //   Topic, auf dem die Arduino-Node ihren State veröffentlicht.
    //   Aus diesem State wird current_avg_a/current_a gelesen.
    this->declare_parameter<std::string>(
      "arduino_state_topic",
      "/battery/arduino_state");

    // voltage_state_topic:
    //   Einziger Output dieser Node.
    this->declare_parameter<std::string>(
      "voltage_state_topic",
      "/battery/robotino_voltage_state");

    // full_voltage_v:
    //   Spannung, die als 100 % betrachtet wird.
    //   Bei 18-V-Werkzeugakkus liegt vollgeladen häufig ungefähr bei 20.5 V.
    this->declare_parameter<double>("full_voltage_v", 20.5);

    // idle_current_threshold_a:
    //   Wenn der Betrag des Stroms kleiner als dieser Wert ist,
    //   betrachten wir den Robotino als "nahe Stillstand".
    this->declare_parameter<double>("idle_current_threshold_a", 0.5);

    // idle_required_s:
    //   Der Strom muss so lange unterhalb des Schwellwerts bleiben,
    //   bevor die Spannung als gültige Ruhespannung veröffentlicht wird.
    this->declare_parameter<double>("idle_required_s", 20.0);

    // poll_period_ms:
    //   Wie oft die Node prüft, ob Stillstand vorliegt und ggf. die Spannung liest.
    this->declare_parameter<int>("poll_period_ms", 1000);

    // clamp_percent:
    //   Begrenze den Spannungs-Prozentwert auf 0...100 %.
    this->declare_parameter<bool>("clamp_percent", true);

    // publish_invalid_state:
    //   true  = State wird auch veröffentlicht, wenn noch kein gültiger Stillstand vorliegt.
    //           Dann steht idle_valid=0 im State.
    //   false = State wird nur bei gültiger Stillstandsmessung veröffentlicht.
    this->declare_parameter<bool>("publish_invalid_state", true);

    // ------------------------------------------------------------
    // Parameterwerte auslesen
    // ------------------------------------------------------------
    robotino_url_ = this->get_parameter("robotino_url").as_string();
    arduino_state_topic_ = this->get_parameter("arduino_state_topic").as_string();
    voltage_state_topic_ = this->get_parameter("voltage_state_topic").as_string();

    full_voltage_v_ = this->get_parameter("full_voltage_v").as_double();
    idle_current_threshold_a_ = this->get_parameter("idle_current_threshold_a").as_double();
    idle_required_s_ = this->get_parameter("idle_required_s").as_double();
    poll_period_ms_ = this->get_parameter("poll_period_ms").as_int();
    clamp_percent_ = this->get_parameter("clamp_percent").as_bool();
    publish_invalid_state_ = this->get_parameter("publish_invalid_state").as_bool();

    // Schutz gegen falschen Parameter.
    if (full_voltage_v_ <= 0.0) {
      RCLCPP_WARN(this->get_logger(), "full_voltage_v <= 0. Setze auf 20.5 V.");
      full_voltage_v_ = 20.5;
    }

    // ------------------------------------------------------------
    // Publisher
    // ------------------------------------------------------------
    // Wir veröffentlichen bewusst nur EIN State-Topic.
    voltage_state_pub_ =
      this->create_publisher<std_msgs::msg::String>(voltage_state_topic_, 10);

    // ------------------------------------------------------------
    // Subscriber
    // ------------------------------------------------------------
    // Wir lesen den Arduino-State, um daraus den Strom zu bekommen.
    // Der Strom entscheidet, ob der Robotino gerade still genug ist,
    // um eine verwertbare Ruhespannung zu messen.
    arduino_state_sub_ =
      this->create_subscription<std_msgs::msg::String>(
        arduino_state_topic_,
        10,
        std::bind(
          &RobotinoVoltageStateReader::arduino_state_callback,
          this,
          std::placeholders::_1));

    // ------------------------------------------------------------
    // libcurl initialisieren
    // ------------------------------------------------------------
    // libcurl wird für HTTP GET benutzt.
    curl_global_init(CURL_GLOBAL_DEFAULT);

    // ------------------------------------------------------------
    // Timer starten
    // ------------------------------------------------------------
    // Der Timer läuft regelmäßig und entscheidet:
    //   1. Haben wir schon Arduino-Stromdaten?
    //   2. Ist der Strom lange genug klein?
    //   3. Wenn ja: Robotino-Spannung lesen und State publishen.
    timer_ =
      this->create_wall_timer(
        std::chrono::milliseconds(poll_period_ms_),
        std::bind(&RobotinoVoltageStateReader::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Robotino Voltage State Reader gestartet.");
    RCLCPP_INFO(this->get_logger(), "Arduino-State Input: %s", arduino_state_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Voltage-State Output: %s", voltage_state_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Robotino URL: %s", robotino_url_.c_str());
  }

  ~RobotinoVoltageStateReader() override
  {
    // libcurl wieder sauber herunterfahren.
    curl_global_cleanup();
  }

private:
  // ------------------------------------------------------------
  // HTTP-Hilfsfunktion für libcurl
  // ------------------------------------------------------------
  // libcurl ruft diese Funktion auf, sobald Daten vom HTTP-Request ankommen.
  // Wir hängen die Daten einfach an einen std::string an.
  static size_t write_callback(void *contents, size_t size, size_t nmemb, void *userp)
  {
    const size_t total_size = size * nmemb;
    auto *buffer = static_cast<std::string *>(userp);
    buffer->append(static_cast<char *>(contents), total_size);
    return total_size;
  }

  // ------------------------------------------------------------
  // Arduino-State parsen
  // ------------------------------------------------------------
  // Der Arduino-State sieht ungefähr so aus:
  //   ARDUINO_BATTERY_STATE;ros_stamp_ns=...;arduino_ms=...;current_a=2.34;current_avg_a=2.12;...
  //
  // Diese Funktion zerlegt den String in key=value Paare.
  std::unordered_map<std::string, std::string> parse_semicolon_key_value_state(
    const std::string &line) const
  {
    std::unordered_map<std::string, std::string> values;

    std::stringstream ss(line);
    std::string token;

    // Erstes Token ist meistens nur der State-Name, z. B. ARDUINO_BATTERY_STATE.
    // Danach folgen key=value Einträge.
    while (std::getline(ss, token, ';')) {
      const auto eq_pos = token.find('=');

      if (eq_pos == std::string::npos) {
        continue;
      }

      const std::string key = token.substr(0, eq_pos);
      const std::string value = token.substr(eq_pos + 1);

      values[key] = value;
    }

    return values;
  }

  // ------------------------------------------------------------
  // Sicheres Auslesen eines float-Wertes aus einem State
  // ------------------------------------------------------------
  bool get_float_from_state(
    const std::unordered_map<std::string, std::string> &values,
    const std::string &key,
    float &output) const
  {
    const auto it = values.find(key);

    if (it == values.end()) {
      return false;
    }

    try {
      output = std::stof(it->second);
      return true;
    }
    catch (...) {
      return false;
    }
  }

  // ------------------------------------------------------------
  // Callback: Arduino-State empfangen
  // ------------------------------------------------------------
  void arduino_state_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    const auto values = parse_semicolon_key_value_state(msg->data);

    float current_from_state = 0.0f;

    // Bevorzugt nutzen wir current_avg_a, weil der Arduino diesen Wert bereits glättet.
    // Falls dieser Key nicht existiert, verwenden wir current_a als Fallback.
    if (!get_float_from_state(values, "current_avg_a", current_from_state)) {
      if (!get_float_from_state(values, "current_a", current_from_state)) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          5000,
          "Arduino-State enthält weder current_avg_a noch current_a.");
        return;
      }
    }

    last_current_a_ = current_from_state;
    current_received_ = true;

    // ------------------------------------------------------------
    // Stillstandserkennung
    // ------------------------------------------------------------
    // Wenn der Strombetrag klein ist, starten wir einen Stillstands-Timer.
    // Wenn der Strom wieder größer wird, setzen wir den Timer zurück.
    const auto now = this->now();

    if (std::fabs(last_current_a_) < idle_current_threshold_a_) {
      if (!idle_timer_running_) {
        idle_start_time_ = now;
        idle_timer_running_ = true;
      }
    } else {
      idle_timer_running_ = false;
      idle_confirmed_ = false;
    }
  }

  // ------------------------------------------------------------
  // Berechne, wie lange der Strom schon unter dem Schwellwert liegt
  // ------------------------------------------------------------
  double get_idle_time_s() const
  {
    if (!idle_timer_running_) {
      return 0.0;
    }

    return (this->now() - idle_start_time_).seconds();
  }

  // ------------------------------------------------------------
  // HTTP GET ausführen
  // ------------------------------------------------------------
  std::string http_get(const std::string &url)
  {
    CURL *curl = curl_easy_init();
    std::string response;

    if (!curl) {
      throw std::runtime_error("curl_easy_init fehlgeschlagen");
    }

    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_callback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);

    // Kurze Timeouts, damit die Node nicht zu lange blockiert,
    // falls der Robotino gerade nicht erreichbar ist.
    curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, 800L);
    curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT_MS, 500L);

    const CURLcode result = curl_easy_perform(curl);

    long http_code = 0;
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);

    curl_easy_cleanup(curl);

    if (result != CURLE_OK) {
      throw std::runtime_error(
        std::string("HTTP request fehlgeschlagen: ") + curl_easy_strerror(result));
    }

    if (http_code != 200 && http_code != 0) {
      std::ostringstream error;
      error << "HTTP Status " << http_code;
      throw std::runtime_error(error.str());
    }

    return response;
  }

  // ------------------------------------------------------------
  // Zahl aus JSON-ähnlicher Robotino-Antwort extrahieren
  // ------------------------------------------------------------
  bool extract_number_for_key(
    const std::string &text,
    const std::string &key,
    double &value) const
  {
    // Sucht z. B.:
    //   "voltage": 23700
    //   "bat_voltage": 23.7
    const std::regex pattern(
      "\\\"" + key + "\\\"\\s*:\\s*(-?[0-9]+(?:\\.[0-9]+)?)");

    std::smatch match;

    if (std::regex_search(text, match, pattern) && match.size() >= 2) {
      value = std::stod(match[1].str());
      return true;
    }

    return false;
  }

  // ------------------------------------------------------------
  // Robotino-Spannung aus HTTP-Antwort lesen
  // ------------------------------------------------------------
  bool parse_voltage_v(const std::string &response, double &voltage_v) const
  {
    // Je nach API/Robotino-Version kann der Key anders heißen.
    const std::vector<std::string> possible_keys = {
      "voltage",
      "bat_voltage",
      "battery_voltage",
      "batteryVoltage"
    };

    double raw_value = 0.0;

    for (const auto &key : possible_keys) {
      if (extract_number_for_key(response, key, raw_value)) {
        // Einige Robotino/Fawkes-Codes verwenden Millivolt:
        //   23700 = 23.7 V
        // Falls der Wert größer als 100 ist, interpretieren wir ihn als mV.
        if (std::fabs(raw_value) > 100.0) {
          voltage_v = raw_value / 1000.0;
        } else {
          voltage_v = raw_value;
        }

        return true;
      }
    }

    return false;
  }

  // ------------------------------------------------------------
  // Spannung relativ zur Vollspannung berechnen
  // ------------------------------------------------------------
  float calculate_voltage_percent(float voltage_v) const
  {
    float percent =
      static_cast<float>((voltage_v / full_voltage_v_) * 100.0);

    if (clamp_percent_) {
      percent = std::max(0.0f, std::min(100.0f, percent));
    }

    return percent;
  }

  // ------------------------------------------------------------
  // State veröffentlichen
  // ------------------------------------------------------------
  void publish_voltage_state(
    bool idle_valid,
    float idle_time_s,
    float voltage_v,
    float voltage_percent)
  {
    std_msgs::msg::String msg;

    std::ostringstream out;

    // Einheitliches, maschinenlesbares Format.
    // Die nächste Berechnungs-Node kann diesen State einfach wieder nach key=value parsen.
    out << "ROBOTINO_VOLTAGE_STATE";
    out << ";ros_stamp_ns=" << this->now().nanoseconds();
    out << ";idle_valid=" << (idle_valid ? 1 : 0);
    out << ";idle_time_s=" << idle_time_s;
    out << ";voltage_v=" << voltage_v;
    out << ";voltage_percent=" << voltage_percent;

    msg.data = out.str();
    voltage_state_pub_->publish(msg);
  }

  // ------------------------------------------------------------
  // Timer-Funktion
  // ------------------------------------------------------------
  void timer_callback()
  {
    // Falls noch kein Arduino-State angekommen ist, können wir noch nicht wissen,
    // ob der Robotino gerade stillsteht.
    if (!current_received_) {
      if (publish_invalid_state_) {
        publish_voltage_state(false, 0.0f, last_voltage_v_, last_voltage_percent_);
      }

      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        5000,
        "Noch kein Arduino-State auf %s empfangen.",
        arduino_state_topic_.c_str());
      return;
    }

    const float idle_time_s =
      static_cast<float>(get_idle_time_s());

    idle_confirmed_ =
      idle_timer_running_ &&
      idle_time_s >= static_cast<float>(idle_required_s_);

    // Wenn noch kein gültiger Stillstand vorliegt, publishen wir optional trotzdem
    // einen State mit idle_valid=0. Dadurch weiß die Berechnungs-Node:
    //   Spannung gerade NICHT für SOC-Korrektur verwenden.
    if (!idle_confirmed_) {
      if (publish_invalid_state_) {
        publish_voltage_state(false, idle_time_s, last_voltage_v_, last_voltage_percent_);
      }
      return;
    }

    try {
      // Robotino-Spannung per REST lesen.
      const std::string response = http_get(robotino_url_);

      double voltage_v_double = 0.0;

      if (!parse_voltage_v(response, voltage_v_double)) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          5000,
          "Keine Spannung im Robotino-Response gefunden.");
        return;
      }

      last_voltage_v_ =
        static_cast<float>(voltage_v_double);

      last_voltage_percent_ =
        calculate_voltage_percent(last_voltage_v_);

      // Nur diese eine State-Zeile wird veröffentlicht.
      publish_voltage_state(
        true,
        idle_time_s,
        last_voltage_v_,
        last_voltage_percent_);

      RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        3000,
        "Robotino-Spannung im Stillstand: %.2f V | %.1f %% | Idle %.1f s",
        last_voltage_v_,
        last_voltage_percent_,
        idle_time_s);
    }
    catch (const std::exception &e) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        5000,
        "Robotino-Spannung konnte nicht gelesen werden: %s",
        e.what());
    }
  }

  // ------------------------------------------------------------
  // Parameter / Konfiguration
  // ------------------------------------------------------------
  std::string robotino_url_;
  std::string arduino_state_topic_;
  std::string voltage_state_topic_;

  double full_voltage_v_ = 20.5;
  double idle_current_threshold_a_ = 0.5;
  double idle_required_s_ = 20.0;
  int poll_period_ms_ = 1000;
  bool clamp_percent_ = true;
  bool publish_invalid_state_ = true;

  // ------------------------------------------------------------
  // Laufzeit-Zustände
  // ------------------------------------------------------------
  bool current_received_ = false;
  float last_current_a_ = 0.0f;

  bool idle_timer_running_ = false;
  bool idle_confirmed_ = false;
  rclcpp::Time idle_start_time_;

  float last_voltage_v_ = 0.0f;
  float last_voltage_percent_ = 0.0f;

  // ------------------------------------------------------------
  // ROS2 Objekte
  // ------------------------------------------------------------
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr arduino_state_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr voltage_state_pub_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotinoVoltageStateReader>());
  rclcpp::shutdown();
  return 0;
}
