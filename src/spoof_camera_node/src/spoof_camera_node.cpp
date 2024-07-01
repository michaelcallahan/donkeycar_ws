#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

class SpoofCameraNode : public rclcpp::Node
{
public:
    SpoofCameraNode()
    : Node("spoof_camera_node")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera_image", 10);
        timer_ = this->create_wall_timer(
            100ms, std::bind(&SpoofCameraNode::timer_callback, this));
    }

private:
    void timer_callback()
    {
        // Generate a synthetic image
        cv::Mat image(480, 640, CV_8UC3, cv::Scalar(0, 0, 255)); // Red image

        // Convert to ROS2 Image message
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
        publisher_->publish(*msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SpoofCameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
