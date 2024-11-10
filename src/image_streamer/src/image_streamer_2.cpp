#include <rclcpp/rclcpp.hpp>
#include <gst/gst.h>
#include <opencv2/opencv.hpp>
#include <thread>

class BlackFrameStreamer : public rclcpp::Node
{
public:
    BlackFrameStreamer() : Node("black_frame_streamer"), pipeline_(nullptr), appsrc_(nullptr)
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
                     "caps", gst_caps_from_string("video/x-raw,format=RGB,width=640,height=480,framerate=30/1"), 
                     NULL);

        RCLCPP_INFO(this->get_logger(), "Start pipeline.");

        // Set pipeline to PLAYING state and check if it successfully transitions
        GstStateChangeReturn ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
        //GstState state, pending;
        //gst_element_get_state(pipeline_, &state, &pending, GST_CLOCK_TIME_NONE);
        //if (ret != GST_STATE_CHANGE_SUCCESS || state != GST_STATE_PLAYING) {
        //    RCLCPP_ERROR(this->get_logger(), "GStreamer pipeline failed to transition to PLAYING. Current state: %d, Pending state: %d", state, pending);
        //} else {
        //    RCLCPP_INFO(this->get_logger(), "GStreamer pipeline is now in PLAYING state.");
        //}

        // Start streaming black frames
        stream_black_frames();
    }

    ~BlackFrameStreamer()
    {
        if (pipeline_) {
            gst_element_set_state(pipeline_, GST_STATE_NULL);
            gst_object_unref(pipeline_);
        }
    }

private:
    void stream_black_frames()
    {
        std::thread([this]() {
            while (rclcpp::ok()) {
                // Create a black frame (480x640, 8-bit, 3-channel)
                cv::Mat frame = cv::Mat::zeros(480, 640, CV_8UC3);

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
                    break;
                } else {
                    RCLCPP_INFO(this->get_logger(), "Buffer pushed to GStreamer pipeline (size: %zu bytes)", frame.total() * frame.elemSize());
                }

                // Stream at ~30 FPS
                std::this_thread::sleep_for(std::chrono::milliseconds(33));
            }
        }).detach();
    }

    GstElement *pipeline_;
    GstElement *appsrc_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BlackFrameStreamer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
