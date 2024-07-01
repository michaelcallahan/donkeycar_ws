#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <libcamera/libcamera.h>
#include <libcamera/camera_manager.h>
#include <libcamera/request.h>
#include <libcamera/framebuffer_allocator.h>
#include <sys/mman.h>

using namespace std::chrono_literals;

class CameraNode : public rclcpp::Node
{
public:
    CameraNode()
    : Node("camera_node")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera_image", 10);
        timer_ = this->create_wall_timer(
            100ms, std::bind(&CameraNode::timer_callback, this));

        camera_manager_ = std::make_unique<libcamera::CameraManager>();
        if (camera_manager_->start() != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to start camera manager");
            rclcpp::shutdown();
        }

        if (camera_manager_->cameras().empty()) {
            RCLCPP_ERROR(this->get_logger(), "No cameras available");
            rclcpp::shutdown();
        }

        camera_ = camera_manager_->get(camera_manager_->cameras()[0]->id());
        if (camera_->acquire()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to acquire camera");
            rclcpp::shutdown();
        }

        configuration_ = camera_->generateConfiguration({libcamera::StreamRole::Viewfinder});
        if (!configuration_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to generate camera configuration");
            rclcpp::shutdown();
        }

        configuration_->at(0).pixelFormat = libcamera::formats::RGB888;
        configuration_->at(0).size = libcamera::Size(640, 480);
        if (camera_->configure(configuration_.get())) {
            RCLCPP_ERROR(this->get_logger(), "Failed to configure camera");
            rclcpp::shutdown();
        }

        allocator_ = std::make_unique<libcamera::FrameBufferAllocator>(camera_);
        for (const auto &stream : *configuration_) {
            if (allocator_->allocate(stream.stream()) < 0) {
                RCLCPP_ERROR(this->get_logger(), "Failed to allocate buffers");
                rclcpp::shutdown();
            }
        }

        for (const auto &stream : *configuration_) {
            for (const std::unique_ptr<libcamera::FrameBuffer> &buffer : allocator_->buffers(stream.stream())) {
                for (const libcamera::FrameBuffer::Plane &plane : buffer->planes()) {
                    void *map = mmap(NULL, plane.length, PROT_READ | PROT_WRITE, MAP_SHARED, plane.fd.get(), 0);
                    if (map == MAP_FAILED) {
                        RCLCPP_ERROR(this->get_logger(), "Failed to map buffer");
                        rclcpp::shutdown();
                    }
                    mapped_buffers_[buffer.get()].push_back(map);
                }
                frame_buffers_.push_back(buffer.get());
            }
        }

        camera_->requestCompleted.connect(this, &CameraNode::requestComplete);

        startCamera();
    }

    ~CameraNode() override
    {
        stopCamera();
        for (auto &pair : mapped_buffers_) {
            for (void *map : pair.second) {
                munmap(map, pair.first->planes()[0].length);
            }
        }
        camera_->release();
        camera_manager_->stop();
    }

private:
    void timer_callback()
    {
        if (!camera_active_) return;

        std::unique_ptr<libcamera::Request> request = camera_->createRequest();
        if (!request) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create request");
            return;
        }

        for (auto buffer : frame_buffers_) {
            if (request->addBuffer(configuration_->at(0).stream(), buffer) < 0) {
                RCLCPP_ERROR(this->get_logger(), "Failed to add buffer to request");
                return;
            }
        }

        if (camera_->queueRequest(request.get()) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to queue request");
        }
    }

    void requestComplete(libcamera::Request *request)
    {
        for (auto &pair : request->buffers()) {
            libcamera::FrameBuffer *buffer = pair.second;
            const libcamera::FrameMetadata &metadata = buffer->metadata();
            if (metadata.status != libcamera::FrameMetadata::FrameSuccess) {
                RCLCPP_ERROR(this->get_logger(), "Frame capture failed");
                return;
            }

            void *mapped_buffer = mapped_buffers_[buffer][0];
            cv::Mat frame(480, 640, CV_8UC3, mapped_buffer);
            auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
            publisher_->publish(*msg);
        }
    }

    void startCamera()
    {
        camera_active_ = (camera_->start() == 0);
    }

    void stopCamera()
    {
        if (camera_active_) {
            camera_->stop();
            camera_active_ = false;
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<libcamera::CameraManager> camera_manager_;
    std::shared_ptr<libcamera::Camera> camera_;
    std::unique_ptr<libcamera::CameraConfiguration> configuration_;
    std::unique_ptr<libcamera::FrameBufferAllocator> allocator_;
    std::vector<libcamera::FrameBuffer *> frame_buffers_;
    std::map<libcamera::FrameBuffer *, std::vector<void *>> mapped_buffers_;
    bool camera_active_ = false;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
