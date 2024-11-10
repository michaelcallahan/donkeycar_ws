extern "C" {
    #include <linux/i2c.h>
    #include <linux/i2c-dev.h>
    #include <i2c/smbus.h>
}

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <gpiod.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <fcntl.h>
#include <sys/ioctl.h>

#define I2C_ADDRESS 0x40  // Define I2C address for PCA9685

bool is_raspberry_pi() {
    std::ifstream cpuinfo("/proc/cpuinfo");
    if (!cpuinfo.is_open()) return false;

    std::string line;
    while (std::getline(cpuinfo, line)) {
        if (line.find("Model") != std::string::npos && line.find("Raspberry Pi") != std::string::npos)
            return true;
    }
    return false;
}

class MotorControlNode : public rclcpp::Node
{
public:
    MotorControlNode()
    : Node("motor_control_node")
    {
        motor_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "motor_speed", 10, std::bind(&MotorControlNode::motor_callback, this, std::placeholders::_1));
        steer_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "steering_angle", 10, std::bind(&MotorControlNode::steer_callback, this, std::placeholders::_1));

        if (is_raspberry_pi()) {
            if (!initialize_i2c()) {
                rclcpp::shutdown();
            } else {
                setPWMFreq(50);  // Set frequency to 50Hz for servo control
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "Not running on a Raspberry Pi. Skipping I2C initialization.");
        }
    }

    ~MotorControlNode() { if (is_raspberry_pi()) close(i2c_fd); }

private:
    bool initialize_i2c() {
        i2c_fd = open("/dev/i2c-1", O_RDWR);
        if (i2c_fd < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open I2C device");
            return false;
        }
        if (ioctl(i2c_fd, I2C_SLAVE, I2C_ADDRESS) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to acquire bus access and/or talk to slave");
            close(i2c_fd);
            return false;
        }
        return true;
    }

    void setPWMFreq(int freq) {
        int prescale_val = static_cast<int>(25000000.0 / (4096 * freq) - 1 + 0.5);
        i2c_smbus_write_byte_data(i2c_fd, 0x00, 0x10);           // Enter sleep mode
        i2c_smbus_write_byte_data(i2c_fd, 0xFE, prescale_val);   // Set prescaler
        i2c_smbus_write_byte_data(i2c_fd, 0x00, 0x80);           // Wake up
        usleep(500);
        i2c_smbus_write_byte_data(i2c_fd, 0x00, 0xA1);           // Auto-increment mode
    }

    void motor_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        motor_speed_ = convert_to_pwm(msg->data, throttle_reverse_, throttle_zero_, throttle_forward_);
        RCLCPP_INFO(this->get_logger(), "Motor speed set to: %d", motor_speed_);
        setPWM(0, 0, motor_speed_); // Channel 0 for motor
    }

    void steer_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        steer_angle_ = convert_to_pwm(msg->data, steer_left_, steer_center_, steer_right_);
        RCLCPP_INFO(this->get_logger(), "Steering angle set to: %d", steer_angle_);
        setPWM(1, 0, steer_angle_); // Channel 1 for steering
    }

    int convert_to_pwm(float value, int min_pwm, int zero_pwm, int max_pwm) {
        value = std::clamp(value, -1.0f, 1.0f);
        if (value < 0)
            return zero_pwm + value * (zero_pwm - min_pwm);
        else
            return zero_pwm + value * (max_pwm - zero_pwm);
    }

    void setPWM(int channel, int on, int off) {
        i2c_smbus_write_byte_data(i2c_fd, 0x06 + 4 * channel, on & 0xFF);     // ON_L
        i2c_smbus_write_byte_data(i2c_fd, 0x07 + 4 * channel, on >> 8);       // ON_H
        i2c_smbus_write_byte_data(i2c_fd, 0x08 + 4 * channel, off & 0xFF);    // OFF_L
        i2c_smbus_write_byte_data(i2c_fd, 0x09 + 4 * channel, off >> 8);      // OFF_H
        RCLCPP_INFO(this->get_logger(), "PWM set on channel %d: on=%d, off=%d", channel, on, off);
    }

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr motor_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr steer_sub_;

    int i2c_fd = -1;
    int steer_angle_ = 0;
    int motor_speed_ = 0;

    int steer_left_ = 300, steer_center_ = 350, steer_right_ = 400;
    int throttle_reverse_ = 300, throttle_zero_ = 350, throttle_forward_ = 400;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
