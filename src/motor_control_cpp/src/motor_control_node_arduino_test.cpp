#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <gpiod.h>
//#include <i2c/smbus.h>
#include <unistd.h>

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <err.h>
#include <errno.h>

#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <string>

// Define I2C address for PCA9685 or Arduino Nano
#define I2C_ADDRESS 0x40

static inline __s32 i2c_smbus_access(int file, char read_write, __u8 command,
                                     int size, union i2c_smbus_data *data)
{
	struct i2c_smbus_ioctl_data args;

	args.read_write = read_write;
	args.command = command;
	args.size = size;
	args.data = data;
	return ioctl(file,I2C_SMBUS,&args);
}


static inline __s32 i2c_smbus_read_byte_data(int file, __u8 command)
{
	union i2c_smbus_data data;
	if (i2c_smbus_access(file,I2C_SMBUS_READ,command,
	                     I2C_SMBUS_BYTE_DATA,&data))
		return -1;
	else
		return 0x0FF & data.byte;
}

static inline __s32 i2c_smbus_write_byte_data(int file, __u8 command, __u8 value)
{
	union i2c_smbus_data data;
	data.byte = value;
	return i2c_smbus_access(file, I2C_SMBUS_WRITE, command,
				I2C_SMBUS_BYTE_DATA, &data);
}

bool is_raspberry_pi() {
    std::ifstream cpuinfo("/proc/cpuinfo");
    if (!cpuinfo.is_open()) {
        return false;
    }

    std::string line;
    while (std::getline(cpuinfo, line)) {
        if (line.find("Model") != std::string::npos && line.find("Raspberry Pi") != std::string::npos) {
            return true;
        }
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
            if (!initialize_gpio() || !initialize_i2c()) {
                rclcpp::shutdown();
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "Not running on a Raspberry Pi. Skipping GPIO initialization.");
        }
    }

    ~MotorControlNode()
    {
        if (is_raspberry_pi() && chip) {
            close(i2c_fd);
            gpiod_chip_close(chip);
        }
    }

private:
    bool initialize_gpio() {
        chip = gpiod_chip_open_by_name("gpiochip0");
        if (!chip) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open gpiochip0");
            return false;
        }
        return true;
    }

    bool initialize_i2c() {
        i2c_fd = open("/dev/i2c-1", O_RDWR);
        if (i2c_fd < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open I2C device");
            return false;
        }

        if (ioctl(i2c_fd, I2C_SLAVE, I2C_ADDRESS) < 0) { // Arduino Nano I2C address
            RCLCPP_ERROR(this->get_logger(), "Failed to acquire bus access and/or talk to slave");
            close(i2c_fd);
            return false;
        }
        return true;
    }

    void motor_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        if (!is_raspberry_pi()) {
            RCLCPP_WARN(this->get_logger(), "Ignoring motor command: not running on Raspberry Pi.");
            return;
        }

        motor_speed_ = convert_to_pwm(msg->data, throttle_reverse_, throttle_zero_, throttle_forward_);
        RCLCPP_INFO(this->get_logger(), "Motor speed set to: %d", motor_speed_);
        //send_i2c_data(steer_angle_, motor_speed_); // Send data to Arduino
        setPWM(0, 0, motor_speed_); // Optionally control motor driver directly
    }

    void steer_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        if (!is_raspberry_pi()) {
            RCLCPP_WARN(this->get_logger(), "Ignoring steer command: not running on Raspberry Pi.");
            return;
        }

        steer_angle_ = convert_to_pwm(msg->data, steer_left_, steer_center_, steer_right_);
        RCLCPP_INFO(this->get_logger(), "Steering angle set to: %d", steer_angle_);
        //send_i2c_data(steer_angle_, motor_speed_); // Send data to Arduino
        setPWM(1, 0, steer_angle_); // Optionally control motor driver directly
    }

    void send_i2c_data(int steer_angle, int motor_speed) {
        uint8_t data[4];
        data[0] = (steer_angle >> 8) & 0xFF;
        data[1] = steer_angle & 0xFF;
        data[2] = (motor_speed >> 8) & 0xFF;
        data[3] = motor_speed & 0xFF;

        if (write(i2c_fd, data, 4) != 4) {
            RCLCPP_ERROR(this->get_logger(), "Failed to send data to Arduino");
        } else {
            RCLCPP_INFO(this->get_logger(), "Sent data to Arduino: steer_angle=%d, motor_speed=%d", steer_angle, motor_speed);
        }
    }

    int convert_to_pwm(float value, int min_pwm, int zero_pwm, int max_pwm)
    {
        if (value < -1.0) value = -1.0;
        if (value > 1.0) value = 1.0;
        if (value < 0)
            return zero_pwm + value * (zero_pwm - min_pwm);
        else
            return zero_pwm + value * (max_pwm - zero_pwm);
    }

    void setPWM(int channel, int on, int off)
    {
        i2c_smbus_write_byte_data(i2c_fd, 0x06 + 4 * channel, on & 0xFF);
        i2c_smbus_write_byte_data(i2c_fd, 0x07 + 4 * channel, on >> 8);
        i2c_smbus_write_byte_data(i2c_fd, 0x08 + 4 * channel, off & 0xFF);
        i2c_smbus_write_byte_data(i2c_fd, 0x09 + 4 * channel, off >> 8);
    }

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr motor_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr steer_sub_;

    int steer_angle_ = 0;
    int motor_speed_ = 0;

    int steer_left_ = 204, steer_center_ = 307, steer_right_ = 410;
    int throttle_reverse_ = 204, throttle_zero_ = 307, throttle_forward_ = 410;

    struct gpiod_chip *chip = nullptr;
    int i2c_fd = -1;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
