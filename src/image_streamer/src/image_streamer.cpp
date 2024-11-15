#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>

class ImageStreamer : public rclcpp::Node
{
public:
    ImageStreamer() : Node("image_streamer")
    {
                // Initialize GStreamer
        gst_init(nullptr, nullptr);

        // Define GStreamer pipeline to stream black frames over UDP to localhost:5000
        std::string pipeline_str = "appsrc name=appsrc ! videoconvert ! x264enc bitrate=500 tune=zerolatency ! rtph264pay config-interval=1 ! udpsink host=127.0.0.1 port=5000 sync=false";

        RCLCPP_INFO(this->get_logger(), "Starting GStreamer.");

        // Parse and create the GStreamer pipeline
        GError *error = nullptr;
        pipeline_ = gst_parse_launch(pipeline_str.c_str(), &error);
        if (!pipeline_ || error) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create GStreamer pipeline: %s", error ? error->message : "Unknown error");
            g_clear_error(&error);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Retrieve appsrc.");

        // Retrieve appsrc from the pipeline
        appsrc_ = gst_bin_get_by_name(GST_BIN(pipeline_), "appsrc");
        g_signal_connect(appsrc_, "need-data", G_CALLBACK(start_feed), this);
        if (!appsrc_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get appsrc element from pipeline");
            gst_object_unref(pipeline_);
            return;
        }

        // Configure appsrc properties for continuous streaming
        g_object_set(G_OBJECT(appsrc_), 
                     "format", GST_FORMAT_TIME, 
                     "is-live", TRUE, 
                     "block", TRUE, 
                     "do-timestamp", TRUE, 
                     "caps", gst_caps_from_string("video/x-raw,format=I420,width=1920,height=1080,framerate=30/1"), 
                     NULL);

        RCLCPP_INFO(this->get_logger(), "Start pipeline.");

        GstStateChangeReturn ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);

        // Subscribe to the image topic
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/image_raw", 10, std::bind(&ImageStreamer::image_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "ImageStreamer node has been started");
    }

    ~ImageStreamer()
    {
        gst_element_set_state(pipeline_, GST_STATE_NULL);
        gst_object_unref(pipeline_);
    }

private:
    static void start_feed(GstElement *source, guint size, ImageStreamer *self)
    {
        self->need_data_ = true;
        RCLCPP_INFO(self->get_logger(), "GStreamer requested more data.");
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (!need_data_)
            return;

        // Convert ROS image to OpenCV format
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Convert the OpenCV image to I420 format required by GStreamer pipeline
        cv::Mat frame;
        cv::cvtColor(cv_ptr->image, frame, cv::COLOR_BGR2YUV_I420);

        // Convert OpenCV frame to GStreamer buffer
        GstBuffer *gst_buffer = gst_buffer_new_allocate(NULL, frame.total() * frame.elemSize(), NULL);
        GstMapInfo map;
        gst_buffer_map(gst_buffer, &map, GST_MAP_WRITE);
        memcpy(map.data, frame.data, frame.total() * frame.elemSize());
        gst_buffer_unmap(gst_buffer, &map);

        // Push the buffer to appsrc
        GstFlowReturn ret;
        g_signal_emit_by_name(appsrc_, "push-buffer", gst_buffer, &ret);
        gst_buffer_unref(gst_buffer);

        if (ret != GST_FLOW_OK) {
            RCLCPP_ERROR(this->get_logger(), "Failed to push buffer to GStreamer pipeline: %d", ret);
        } else {
            RCLCPP_INFO(this->get_logger(), "Buffer pushed to GStreamer pipeline (size: %zu bytes)", frame.total() * frame.elemSize());
        }

        // Stream at ~30 FPS
        //std::this_thread::sleep_for(std::chrono::milliseconds(33));
    }

    std::string host_;
    int port_;
    GstElement *pipeline_;
    GstElement *appsrc_;
    bool need_data_ = false;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    gst_init(&argc, &argv);
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageStreamer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
