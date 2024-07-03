#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <libcamera/libcamera.h>
#include <libcamera/camera_manager.h>
#include <libcamera/camera.h>
#include <libcamera/request.h>
#include <libcamera/framebuffer.h>
#include <libcamera/stream.h>
#include <libcamera/control_ids.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <sys/mman.h>

class CameraNode : public rclcpp::Node {
public:
    CameraNode() : Node("camera_node"), camera_manager_(nullptr), camera_(nullptr), camera_started_(false) {
        // Initialize Camera Manager
        RCLCPP_INFO(this->get_logger(), "Initializing Camera Manager");
        camera_manager_ = std::make_shared<libcamera::CameraManager>();
        if (camera_manager_->start() != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to start CameraManager");
            return;
        }

        // Log the libcamera version
        RCLCPP_INFO(this->get_logger(), "libcamera version: %s", LIBCAMERA_VERSION_MAJOR);

        // Get the first available camera
        RCLCPP_INFO(this->get_logger(), "Getting available cameras");
        if (camera_manager_->cameras().empty()) {
            RCLCPP_ERROR(this->get_logger(), "No cameras available");
            return;
        }

        camera_ = camera_manager_->get(camera_manager_->cameras()[0]->id());
        if (!camera_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get camera");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Acquiring camera");
        if (camera_->acquire() != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to acquire camera");
            return;
        }

        // Configure streams
        RCLCPP_INFO(this->get_logger(), "Configuring streams");
        configure_streams();

        // Initialize ROS2 publisher
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image", 10);

        // Set timer callback for capturing frames
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&CameraNode::timer_callback, this));

        // Connect request completed signal to our callback
        camera_->requestCompleted.connect(this, &CameraNode::requestComplete);
    }

    ~CameraNode() {
        RCLCPP_INFO(this->get_logger(), "Shutting down CameraNode");
        if (camera_) {
            RCLCPP_INFO(this->get_logger(), "Stopping camera");
            camera_->stop();
            RCLCPP_INFO(this->get_logger(), "Releasing camera");
            camera_->release();
        }
        if (camera_manager_) {
            RCLCPP_INFO(this->get_logger(), "Stopping CameraManager");
            camera_manager_->stop();
        }
    }

private:
    void configure_streams() {
        RCLCPP_INFO(this->get_logger(), "Generating camera configuration");
        auto config = camera_->generateConfiguration({libcamera::StreamRole::Viewfinder});
        if (config->size() == 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to generate configuration");
            return;
        }

        auto &streamConfig = config->at(0);
        streamConfig.size.width = 1296;
        streamConfig.size.height = 972;
        streamConfig.pixelFormat = libcamera::formats::YUV420;
        streamConfig.bufferCount = 4;

        RCLCPP_INFO(this->get_logger(), "Configuring camera");
        if (camera_->configure(config.get()) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to configure camera");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Allocating frame buffers");
        auto allocator = std::make_unique<libcamera::FrameBufferAllocator>(camera_);
        if (allocator->allocate(streamConfig.stream()) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to allocate buffers");
            return;
        }

        const std::vector<std::unique_ptr<libcamera::FrameBuffer>> &buffers = allocator->buffers(streamConfig.stream());
        for (const std::unique_ptr<libcamera::FrameBuffer> &buffer : buffers) {
            frame_buffers_.push_back(buffer.get());
        }

        RCLCPP_INFO(this->get_logger(), "Creating requests");
        for (libcamera::FrameBuffer *buffer : frame_buffers_) {
            std::unique_ptr<libcamera::Request> request = camera_->createRequest();
            if (!request) {
                RCLCPP_ERROR(this->get_logger(), "Failed to create request");
                return;
            }

            if (request->addBuffer(streamConfig.stream(), buffer) != 0) {
                RCLCPP_ERROR(this->get_logger(), "Failed to add buffer to request");
                return;
            }

            requests_.push_back(std::move(request));
        }
    }

    void timer_callback() {
        if (!camera_started_) {
            RCLCPP_INFO(this->get_logger(), "Starting camera");
            if (camera_->start() != 0) {
                RCLCPP_ERROR(this->get_logger(), "Failed to start camera");
                return;
            }
            camera_started_ = true;
        }
        RCLCPP_INFO(this->get_logger(), "Queueing request");
        for (auto &request : requests_) {
            if (camera_->queueRequest(request.get()) != 0) {
                RCLCPP_ERROR(this->get_logger(), "Failed to queue request");
                return;
            }
        }
    }

    void requestComplete(libcamera::Request *request) {
        RCLCPP_INFO(this->get_logger(), "Request complete");
        if (request->status() == libcamera::Request::RequestComplete) {
            auto buffer = request->buffers().begin()->second;
            const libcamera::FrameBuffer::Plane &plane = buffer->planes()[0];

            // Map the buffer to access the data
            RCLCPP_INFO(this->get_logger(), "Mapping buffer memory");
            void *image_data = mmap(NULL, plane.length, PROT_READ | PROT_WRITE, MAP_SHARED, plane.fd.get(), 0);
            if (image_data == MAP_FAILED) {
                RCLCPP_ERROR(this->get_logger(), "Failed to map buffer memory");
                return;
            }

            RCLCPP_INFO(this->get_logger(), "Buffer memory mapped at %p", image_data);

            cv::Mat frame(972, 1296, CV_8UC3, image_data);

            // Convert OpenCV image to ROS2 message
            auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
            image_pub_->publish(*msg);

            // Unmap the buffer
            RCLCPP_INFO(this->get_logger(), "Unmapping buffer memory");
            if (munmap(image_data, plane.length) != 0) {
                RCLCPP_ERROR(this->get_logger(), "Failed to unmap buffer memory");
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Request failed with status: %d", request->status());
        }
    }

    std::shared_ptr<libcamera::CameraManager> camera_manager_;
    std::shared_ptr<libcamera::Camera> camera_;
    std::vector<libcamera::FrameBuffer *> frame_buffers_;
    std::vector<std::unique_ptr<libcamera::Request>> requests_;
    bool camera_started_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
