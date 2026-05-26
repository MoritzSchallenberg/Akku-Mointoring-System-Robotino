/***************************************************************************
 *  arduino_battery_state_reader.cpp - ROS2 Read in Ardunio Akku Monitoring
 *
 *  Created: May 18 12:00:00 2026
 *  Copyright  2026-2029  Moritz Schallenberg

 *  Aufgabe dieser Node:
 *  - Arduino per USB/Serial lesen
 *  - DATA-Zeilen vom Arduino parsen
 *  - alle Werte in einem gemeinsamen Battery-State-Topic veröffentlichen
 *  - optional wichtige Legacy-Topics weiter veröffentlichen, damit ältere Nodes
 *    wie die Robotino-Spannungsmessung weiterhin funktionieren können
 *
 *  Erwartete Arduino-Zeile:
 *  DATA;ms=12345;dt_ms=1000;current_a=2.34;current_avg_a=2.12;temp_avg_c=28.6;battery_count=4;capacity_ah=16.00;battery_mode=fixed_4_batteries
 
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

#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>

using namespace std::chrono_literals;

// -----------------------------------------------------------------------------
// Kleine Datenstruktur für den kompletten Zustand
// -----------------------------------------------------------------------------
struct ArduinoBatteryState
{
    // Zeitstempel vom Arduino in Millisekunden seit Arduino-Start.
    int arduino_ms = 0;

    // Messintervall des Arduino in Millisekunden.
    int dt_ms = 0;

    // Momentanstrom in Ampere.
    float current_a = 0.0f;

    // Durchschnittsstrom in Ampere, direkt vom Arduino berechnet.
    float current_avg_a = 0.0f;

    // Durchschnittstemperatur der Akku-Temperaturmessungen in Grad Celsius.
    float temp_avg_c = 0.0f;

    // Anzahl der Akkus. Aktuell im Arduino fest auf 4 gesetzt.
    int battery_count = 4;

    // Gesamtkapazität in Ah. Aktuell im Arduino 4 Akkus * 4 Ah = 16 Ah.
    float capacity_ah = 16.0f;

    // Betriebsmodus als Text, z. B. "fixed_4_batteries".
    std::string battery_mode = "unknown";
};

class ArduinoBatteryStateReader : public rclcpp::Node
{
public:
    ArduinoBatteryStateReader()
    : Node("arduino_battery_state_reader")
    {
        // ---------------------------------------------------------------------
        // Parameter
        // ---------------------------------------------------------------------
        // port:
        //   USB-Port des Arduino.
        // baudrate:
        //   Muss zur Serial.begin(...) Baudrate im Arduino-Code passen.
        // read_interval_ms:
        //   Wie oft diese Node versucht, neue Serial-Daten einzulesen.
        // publish_raw_data:
        //   Wenn true, wird jede empfangene Zeile zusätzlich als Debug-Topic veröffentlicht.
        // publish_legacy_topics:
        //   Wenn true, werden zusätzlich einzelne alte Topics veröffentlicht, damit andere Nodes nicht sofort angepasst werden müssen.
        // ---------------------------------------------------------------------
        this->declare_parameter<std::string>("port", "/dev/ttyACM0");
        this->declare_parameter<int>("baudrate", 115200);
        this->declare_parameter<int>("read_interval_ms", 50);
        this->declare_parameter<bool>("publish_raw_data", false);
        this->declare_parameter<bool>("publish_legacy_topics", true);

        port_ = this->get_parameter("port").as_string();
        baudrate_ = this->get_parameter("baudrate").as_int();
        publish_raw_data_ = this->get_parameter("publish_raw_data").as_bool();
        publish_legacy_topics_ = this->get_parameter("publish_legacy_topics").as_bool();

        const int read_interval_ms = this->get_parameter("read_interval_ms").as_int();

        // ---------------------------------------------------------------------
        // Haupt-State-Topic
        // ---------------------------------------------------------------------
        state_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/battery/arduino_state", 10);

        // ---------------------------------------------------------------------
        // Optionales Rohdaten-Topic nur für Debugging
        // ---------------------------------------------------------------------
        raw_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/battery/arduino_raw", 10);

        // ---------------------------------------------------------------------
        // Optional: alte einzelne Topics weiterhin publishen.
        // ---------------------------------------------------------------------
        current_pub_ = this->create_publisher<std_msgs::msg::Float32>(
            "/battery/current", 10);

        current_avg_pub_ = this->create_publisher<std_msgs::msg::Float32>(
            "/battery/current_avg", 10);

        temp_avg_pub_ = this->create_publisher<std_msgs::msg::Float32>(
            "/battery/temp_avg_c", 10);

        battery_count_pub_ = this->create_publisher<std_msgs::msg::Int32>(
            "/battery/count", 10);

        capacity_pub_ = this->create_publisher<std_msgs::msg::Float32>(
            "/battery/capacity_ah", 10);

        // Serielle Schnittstelle öffnen und konfigurieren.
        setup_serial();

        // Timer: ruft regelmäßig read_serial() auf.
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(read_interval_ms),
            std::bind(&ArduinoBatteryStateReader::read_serial, this));

        RCLCPP_INFO(this->get_logger(), "Arduino Battery State Reader gestartet");
        RCLCPP_INFO(this->get_logger(), "Port: %s | Baudrate: %d", port_.c_str(), baudrate_);
        RCLCPP_INFO(this->get_logger(), "State Topic: /battery/arduino_state");
    }

    ~ArduinoBatteryStateReader()
    {
        // Beim Beenden sauber den Serial-Port schließen.
        if (serial_port_ >= 0)
        {
            close(serial_port_);
        }
    }

private:
    // -------------------------------------------------------------------------
    // Serial-/ROS-Variablen
    // -------------------------------------------------------------------------
    int serial_port_ = -1;
    std::string port_;
    int baudrate_ = 115200;
    bool publish_raw_data_ = false;
    bool publish_legacy_topics_ = true;

    // Buffer für serielle Daten.
    std::string serial_buffer_;

    rclcpp::TimerBase::SharedPtr timer_;

    // Publisher für den gemeinsamen State.
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;

    // Debug-Publisher für Rohdaten.
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr raw_pub_;

    // Legacy-/Kompatibilitäts-Publisher.
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr current_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr current_avg_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr temp_avg_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr battery_count_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr capacity_pub_;

    // -------------------------------------------------------------------------
    // Baudrate von normaler Zahl auf Linux-Konstante abbilden.
    // -------------------------------------------------------------------------
    speed_t get_baudrate_constant(int baudrate)
    {
        switch (baudrate)
        {
            case 9600: return B9600;
            case 19200: return B19200;
            case 38400: return B38400;
            case 57600: return B57600;
            case 115200: return B115200;
            default:
                RCLCPP_WARN(
                    this->get_logger(),
                    "Nicht unterstützte Baudrate %d, nutze 115200",
                    baudrate);
                return B115200;
        }
    }

    // -------------------------------------------------------------------------
    // Serial-Port öffnen und so einstellen, dass wir rohe Daten bekommen.
    // -------------------------------------------------------------------------
    void setup_serial()
    {
        // O_NONBLOCK sorgt dafür, dass read() nicht hängen bleibt, wenn gerade
        serial_port_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

        if (serial_port_ < 0)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Konnte Serial Port nicht öffnen: %s",
                port_.c_str());
            return;
        }

        termios tty{};

        if (tcgetattr(serial_port_, &tty) != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "tcgetattr fehlgeschlagen");
            close(serial_port_);
            serial_port_ = -1;
            return;
        }

        speed_t speed = get_baudrate_constant(baudrate_);
        cfsetispeed(&tty, speed);
        cfsetospeed(&tty, speed);

        // 8 Datenbits, keine Parität, 1 Stopbit: klassisch 8N1.
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;

        // Keine Hardware-Flusskontrolle.
        tty.c_cflag &= ~CRTSCTS;

        // Lesen erlauben, lokale Verbindung.
        tty.c_cflag |= CREAD | CLOCAL;

        // Raw mode: keine Terminal-Verarbeitung, kein Echo, keine Signale.
        tty.c_lflag &= ~ICANON;
        tty.c_lflag &= ~ECHO;
        tty.c_lflag &= ~ECHOE;
        tty.c_lflag &= ~ECHONL;
        tty.c_lflag &= ~ISIG;

        // Keine Software-Flusskontrolle und keine Zeilenumwandlung.
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

        // Keine Ausgabe-Verarbeitung.
        tty.c_oflag &= ~OPOST;
        tty.c_oflag &= ~ONLCR;

        // Nicht-blockierendes Verhalten.
        tty.c_cc[VTIME] = 0;
        tty.c_cc[VMIN] = 0;

        if (tcsetattr(serial_port_, TCSANOW, &tty) != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "tcsetattr fehlgeschlagen");
            close(serial_port_);
            serial_port_ = -1;
            return;
        }

        // Alte Daten im Buffer verwerfen.
        tcflush(serial_port_, TCIOFLUSH);
    }

    // -------------------------------------------------------------------------
    // Liest alles, was gerade am Serial-Port verfügbar ist.
    // -------------------------------------------------------------------------
    void read_serial()
    {
        if (serial_port_ < 0)
        {
            return;
        }

        char buffer[256];
        const int n = read(serial_port_, buffer, sizeof(buffer));

        if (n > 0)
        {
            serial_buffer_.append(buffer, n);

            // Prüfen, ob vollständige Zeilen angekommen sind.
            process_buffer();
        }
        else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
        {
            RCLCPP_WARN(this->get_logger(), "Serial read Fehler: errno=%d", errno);
        }
    }

    // -------------------------------------------------------------------------
    // Aus dem Sammelbuffer vollständige Zeilen extrahieren.
    // -------------------------------------------------------------------------
    void process_buffer()
    {
        size_t newline_pos;

        // Solange ein \n im Buffer ist, haben wir mindestens eine vollständige Zeile.
        while ((newline_pos = serial_buffer_.find('\n')) != std::string::npos)
        {
            std::string line = serial_buffer_.substr(0, newline_pos);
            serial_buffer_.erase(0, newline_pos + 1);

            // Windows/Arduino-Terminals senden manchmal \r\n.
            // Das \r entfernen wir hier.
            if (!line.empty() && line.back() == '\r')
            {
                line.pop_back();
            }

            if (!line.empty())
            {
                parse_line(line);
            }
        }

        // Falls durch defekte Daten nie ein \n kommt, würde der Buffer endlos wachsen. Das verhindern wir hier.
        if (serial_buffer_.size() > 1024)
        {
            RCLCPP_WARN(this->get_logger(), "Serial Buffer zu groß, wird geleert");
            serial_buffer_.clear();
        }
    }

    // -------------------------------------------------------------------------
    // key=value-Zeile in eine Map umwandeln.
    // -------------------------------------------------------------------------
    std::unordered_map<std::string, std::string> parse_key_value_line(
        const std::string & line)
    {
        std::unordered_map<std::string, std::string> values;
        std::stringstream ss(line);
        std::string part;

        // Erstes Feld ist "DATA". Das brauchen wir nicht als key=value.
        std::getline(ss, part, ';');

        while (std::getline(ss, part, ';'))
        {
            const size_t eq_pos = part.find('=');

            if (eq_pos == std::string::npos)
            {
                continue;
            }

            const std::string key = part.substr(0, eq_pos);
            const std::string value = part.substr(eq_pos + 1);

            values[key] = value;
        }

        return values;
    }

    // -------------------------------------------------------------------------
    // Hilfsfunktion: float aus der Map lesen.
    // Gibt false zurück, wenn Feld fehlt oder nicht umwandelbar ist.
    // -------------------------------------------------------------------------
    bool get_float(
        const std::unordered_map<std::string, std::string> & values,
        const std::string & key,
        float & output)
    {
        auto it = values.find(key);

        if (it == values.end())
        {
            return false;
        }

        try
        {
            output = std::stof(it->second);
            return true;
        }
        catch (...)
        {
            return false;
        }
    }

    // -------------------------------------------------------------------------
    // Hilfsfunktion: int aus der Map lesen.
    // -------------------------------------------------------------------------
    bool get_int(
        const std::unordered_map<std::string, std::string> & values,
        const std::string & key,
        int & output)
    {
        auto it = values.find(key);

        if (it == values.end())
        {
            return false;
        }

        try
        {
            output = std::stoi(it->second);
            return true;
        }
        catch (...)
        {
            return false;
        }
    }

    // -------------------------------------------------------------------------
    // Die empfangenen Werte in unsere ArduinoBatteryState-Struktur schreiben.
    // -------------------------------------------------------------------------
    bool parse_state(
        const std::unordered_map<std::string, std::string> & values,
        ArduinoBatteryState & state)
    {
        bool ok = true;

        // Pflichtfelder: Diese Werte erwarten wir von deinem neuen Arduino-Code.
        ok &= get_int(values, "ms", state.arduino_ms);
        ok &= get_int(values, "dt_ms", state.dt_ms);
        ok &= get_float(values, "current_a", state.current_a);
        ok &= get_float(values, "current_avg_a", state.current_avg_a);
        ok &= get_float(values, "temp_avg_c", state.temp_avg_c);
        ok &= get_int(values, "battery_count", state.battery_count);
        ok &= get_float(values, "capacity_ah", state.capacity_ah);

        // battery_mode ist kein Zahlenwert, daher lesen wir ihn direkt.
        auto mode_it = values.find("battery_mode");
        if (mode_it != values.end())
        {
            state.battery_mode = mode_it->second;
        }
        else
        {
            state.battery_mode = "unknown";
        }

        return ok;
    }

    // -------------------------------------------------------------------------
    // State-Struktur wieder als kompakte key=value-Zeile ausgeben.
    // -------------------------------------------------------------------------
    std::string make_state_string(const ArduinoBatteryState & state)
    {
        std::ostringstream out;

        // ROS-Zeitstempel ergänzen, damit spätere Nodes wissen, wann diese
        // Zeile im ROS-System angekommen ist.
        const int64_t ros_stamp_ns = this->now().nanoseconds();

        out << "ARDUINO_BATTERY_STATE";
        out << ";ros_stamp_ns=" << ros_stamp_ns;
        out << ";arduino_ms=" << state.arduino_ms;
        out << ";dt_ms=" << state.dt_ms;
        out << ";current_a=" << state.current_a;
        out << ";current_avg_a=" << state.current_avg_a;
        out << ";temp_avg_c=" << state.temp_avg_c;
        out << ";battery_count=" << state.battery_count;
        out << ";capacity_ah=" << state.capacity_ah;
        out << ";battery_mode=" << state.battery_mode;

        return out.str();
    }

    // -------------------------------------------------------------------------
    // Einzelne Legacy-Topics publishen.
    // -------------------------------------------------------------------------
    void publish_legacy_topics(const ArduinoBatteryState & state)
    {
        if (!publish_legacy_topics_)
        {
            return;
        }

        std_msgs::msg::Float32 float_msg;
        std_msgs::msg::Int32 int_msg;

        // Momentanstrom für die Robotino-Spannungs-Node.
        float_msg.data = state.current_a;
        current_pub_->publish(float_msg);

        // Durchschnittsstrom
        float_msg.data = state.current_avg_a;
        current_avg_pub_->publish(float_msg);

        // Durchschnittstemperatur in Grad Celsius.
        float_msg.data = state.temp_avg_c;
        temp_avg_pub_->publish(float_msg);

        // Feste Akkuzahl.
        int_msg.data = state.battery_count;
        battery_count_pub_->publish(int_msg);

        // Gesamtkapazität.
        float_msg.data = state.capacity_ah;
        capacity_pub_->publish(float_msg);
    }

    // -------------------------------------------------------------------------
    // Eine vollständige Zeile vom Arduino auswerten.
    // -------------------------------------------------------------------------
    void parse_line(const std::string & line)
    {
        // Optional: Rohdaten publishen, damit man bei Problemen direkt sieht, was der Arduino wirklich gesendet hat.
        if (publish_raw_data_)
        {
            std_msgs::msg::String raw_msg;
            raw_msg.data = line;
            raw_pub_->publish(raw_msg);
        }
        // START/OFFSET/Debug-Zeilen vom Arduino werden ignoriert.
        if (line.rfind("DATA;", 0) != 0)
        {
            return;
        }
        const auto values = parse_key_value_line(line);
        ArduinoBatteryState state;
        // Wenn Pflichtfelder fehlen, wird die Zeile nicht veröffentlicht.
        // Das verhindert, dass halbe oder defekte Daten in ROS landen.
        if (!parse_state(values, state))
        {
            RCLCPP_WARN(
                this->get_logger(),
                "DATA-Zeile konnte nicht vollständig geparst werden: %s",
                line.c_str());
            return;
        }

        // Gemeinsamen State publishen.
        std_msgs::msg::String state_msg;
        state_msg.data = make_state_string(state);
        state_pub_->publish(state_msg);

        // Optional zusätzlich alte einzelne Topics publishen.
        publish_legacy_topics(state);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArduinoBatteryStateReader>());
    rclcpp::shutdown();
    return 0;
}
