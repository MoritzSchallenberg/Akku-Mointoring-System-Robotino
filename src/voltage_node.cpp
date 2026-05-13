#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

class VoltageNode : public rclcpp::Node
{
public:
    VoltageNode()
    : Node("voltage_node")
    {
        voltage_pub_ =
            this->create_publisher<std_msgs::msg::Float32>(
                "/battery/voltage",
                10);

        timer_ = this->create_wall_timer(
            500ms,
            std::bind(&VoltageNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(),
                    "Voltage Node gestartet");
    }

private:
    float read_voltage()
    {
        /*
         * HIER später:
         * echten Robotino Spannungscode einfügen
         */

        // TESTWERT
        return 23.7f;
    }

    void timer_callback()
    {
        float voltage = read_voltage();

        std_msgs::msg::Float32 msg;
        msg.data = voltage;

        voltage_pub_->publish(msg);

        RCLCPP_INFO(this->get_logger(),
                    "Spannung: %.2f V",
                    voltage);
    }

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr voltage_pub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<VoltageNode>());

    rclcpp::shutdown();

    return 0;
}