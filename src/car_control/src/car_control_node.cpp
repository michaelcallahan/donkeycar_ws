#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float32.hpp>

class CarControlNode : public rclcpp::Node
{
public:
    CarControlNode() : Node("car_control_node")
    {
        joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&CarControlNode::joy_callback, this, std::placeholders::_1));

        speed_publisher_ = this->create_publisher<std_msgs::msg::Float32>("motor_speed", 10);
        steer_publisher_ = this->create_publisher<std_msgs::msg::Float32>("steering_angle", 10);

        RCLCPP_INFO(this->get_logger(), "CarControlNode has been started");
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        auto speed_msg = std_msgs::msg::Float32();
        auto steer_msg = std_msgs::msg::Float32();

        // Assuming left stick vertical axis is for speed and horizontal axis is for steering
        speed_msg.data = msg->axes[1]; // Adjust according to your joystick configuration
        steer_msg.data = msg->axes[0]; // Adjust according to your joystick configuration

        speed_publisher_->publish(speed_msg);
        steer_publisher_->publish(steer_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr speed_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steer_publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CarControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
