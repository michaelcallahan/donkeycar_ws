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
        // Parameters for GStreamer pipeline
        this->declare_parameter<std::string>("host", "127.0.0.1");
        this->declare_parameter<int>("port", 5000);

        this->get_parameter("host", host_);
        this->get_parameter("port", port_);

        std::string pipeline_str = "appsrc name=appsrc ! videoconvert ! video/x-raw,format=I420 ! x264enc bitrate=500 tune=zerolatency ! rtph264pay config-interval=1 ! udpsink host=" + host_ + " port=" + std::to_string(port_) + " sync=false async=false";


        pipeline_ = gst_parse_launch(pipeline_str.c_str(), NULL);
        if (!pipeline_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create GStreamer pipeline");
            rclcpp::shutdown();
            return;
        }

        appsrc_ = gst_bin_get_by_name(GST_BIN(pipeline_), "appsrc");
        if (!appsrc_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get appsrc element from pipeline");
            gst_object_unref(pipeline_);
            rclcpp::shutdown();
            return;
        }

        // Set appsrc properties
        g_object_set(G_OBJECT(appsrc_), "format", GST_FORMAT_TIME, NULL);
        g_signal_connect(appsrc_, "need-data", G_CALLBACK(start_feed), this);

        // Start the GStreamer pipeline
        gst_element_set_state(pipeline_, GST_STATE_PLAYING);

        GstStateChangeReturn ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
        if (ret != GST_STATE_CHANGE_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "GStreamer pipeline failed to transition to PLAYING");
        } else {
            RCLCPP_INFO(this->get_logger(), "GStreamer pipeline is now PLAYING");
        }


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

        // Encode the frame to pass it as a buffer to appsrc
        GstBuffer *gst_buffer = gst_buffer_new_allocate(NULL, frame.total() * frame.elemSize(), NULL);
        GstMapInfo map;
        gst_buffer_map(gst_buffer, &map, GST_MAP_WRITE);
        memcpy(map.data, frame.data, frame.total() * frame.elemSize());
        gst_buffer_unmap(gst_buffer, &map);

        // Push the buffer to GStreamer appsrc
        GstFlowReturn ret;
        g_signal_emit_by_name(appsrc_, "push-buffer", gst_buffer, &ret);
        gst_buffer_unref(gst_buffer);

        if (ret != GST_FLOW_OK) {
            RCLCPP_ERROR(this->get_logger(), "Failed to push buffer to GStreamer pipeline");
            need_data_ = false;
        } else {
            RCLCPP_INFO(this->get_logger(), "Buffer successfully pushed to GStreamer pipeline");
        }
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
    gst_init(&argc, &argv);
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageStreamer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
