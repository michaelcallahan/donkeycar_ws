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
        this->declare_parameter<std::string>("host", "127.0.0.1");
        this->declare_parameter<int>("port", 5000);

        this->get_parameter("host", host_);
        this->get_parameter("port", port_);

        std::string pipeline_str = "appsrc name=appsrc ! videoconvert ! "
                                   "video/x-raw,format=I420 ! x264enc bitrate=500 ! rtph264pay ! "
                                   "udpsink host=" + host_ + " port=" + std::to_string(port_);

        pipeline_ = gst_parse_launch(pipeline_str.c_str(), NULL);
        if (!pipeline_)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to create GStreamer pipeline");
            rclcpp::shutdown();
        }

        appsrc_ = gst_bin_get_by_name(GST_BIN(pipeline_), "appsrc");
        if (!appsrc_)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to get appsrc element from pipeline");
            rclcpp::shutdown();
        }

        g_signal_connect(appsrc_, "need-data", G_CALLBACK(start_feed), this);

        gst_element_set_state(pipeline_, GST_STATE_PLAYING);

        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10, std::bind(&ImageStreamer::image_callback, this, std::placeholders::_1));

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
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (!need_data_)
            return;

        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat frame = cv_ptr->image;
        std::vector<uchar> buffer;
        cv::imencode(".jpg", frame, buffer);

        GstBuffer *gst_buffer = gst_buffer_new_allocate(NULL, buffer.size(), NULL);
        gst_buffer_fill(gst_buffer, 0, buffer.data(), buffer.size());

        GstFlowReturn ret;
        g_signal_emit_by_name(appsrc_, "push-buffer", gst_buffer, &ret);
        gst_buffer_unref(gst_buffer);

        if (ret != GST_FLOW_OK)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to push buffer to GStreamer pipeline");
            need_data_ = false;
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
