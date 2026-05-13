#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

#include <string>
#include <sstream>

class SerialBatteryNode : public rclcpp::Node
{
public:

    SerialBatteryNode()
    : Node("serial_battery_node")
    {
        RCLCPP_INFO(this->get_logger(),
                    "Battery Serial Node gestartet");

        current_pub_1_ =
            this->create_publisher<std_msgs::msg::Float32>(
                "/battery/current_1", 10);

        current_pub_2_ =
            this->create_publisher<std_msgs::msg::Float32>(
                "/battery/current_2", 10);

        temp_pub_ =
            this->create_publisher<std_msgs::msg::String>(
                "/battery/temperatures", 10);

        setup_serial();

        timer_ =
            this->create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(
                    &SerialBatteryNode::read_serial,
                    this));
    }

private:

    int serial_port_;

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr current_pub_1_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr current_pub_2_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr temp_pub_;

    void setup_serial()
    {
        serial_port_ =
            open("/dev/ttyACM1", O_RDWR | O_NOCTTY);

        if(serial_port_ < 0)
        {
            RCLCPP_ERROR(this->get_logger(),
                         "Konnte Serial Port nicht öffnen");
            return;
        }

        termios tty{};

        tcgetattr(serial_port_, &tty);

        cfsetispeed(&tty, B115200);
        cfsetospeed(&tty, B115200);

        tty.c_cflag |= CREAD | CLOCAL;

        tcsetattr(serial_port_, TCSANOW, &tty);
    }

    void read_serial()
    {
        char buffer[256];

        int n = read(serial_port_, buffer, sizeof(buffer));

        if(n > 0)
        {
            std::string line(buffer, n);

            RCLCPP_INFO(this->get_logger(),
                        "RAW: %s",
                        line.c_str());

            parse_line(line);
        }
    }

    void parse_line(const std::string & line)
    {
        if(line.find("CURRENT") != std::string::npos)
        {
            // Beispiel:
            // CURRENT;a0=2.5;a1=2.4

            try
            {
                size_t pos1 = line.find("a0=");
                size_t pos2 = line.find(";a1=");

                float current1 =
                    std::stof(
                        line.substr(
                            pos1 + 3,
                            pos2 - (pos1 + 3)));

                float current2 =
                    std::stof(
                        line.substr(
                            pos2 + 4));

                std_msgs::msg::Float32 msg1;
                msg1.data = current1;

                std_msgs::msg::Float32 msg2;
                msg2.data = current2;

                current_pub_1_->publish(msg1);
                current_pub_2_->publish(msg2);
            }
            catch(...)
            {
                RCLCPP_WARN(this->get_logger(),
                            "Fehler beim Parsen");
            }
        }

        else if(line.find("TEMP") != std::string::npos)
        {
            std_msgs::msg::String msg;

            msg.data = line;

            temp_pub_->publish(msg);
        }
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<SerialBatteryNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}