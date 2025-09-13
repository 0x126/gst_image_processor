#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/header.hpp>
#include <image_transport/image_transport.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <cv_bridge/cv_bridge.h>
#include "v4l2_processor.h"
#include <opencv2/opencv.hpp>
#include <chrono>

namespace gst_image_processor {

class GstImageProcessorNode : public rclcpp::Node {
public:
    explicit GstImageProcessorNode(const rclcpp::NodeOptions& options) 
        : Node("gst_image_processor_node", options) {
        // V4L2 source parameters
        this->declare_parameter("device", "/dev/video0");
        this->declare_parameter("width", 2880);
        this->declare_parameter("height", 1860);
        this->declare_parameter("framerate", 20);
        this->declare_parameter("format", "UYVY");
        
        // Processing parameters
        this->declare_parameter("jpeg_quality", 90);
        this->declare_parameter("buffer_size", 8388608);
        this->declare_parameter("max_buffers", 3);
        
        // Hardware acceleration
        this->declare_parameter("use_nvmm", false);
        this->declare_parameter("io_mode", 0);
        
        // Output parameters
        this->declare_parameter("publish_raw", false);
        this->declare_parameter("publish_compressed", true);
        this->declare_parameter("frame_id", "camera");
        this->declare_parameter("camera_info_url", "");
        
        // Timestamp handling
        this->declare_parameter("use_v4l2_timestamps", true);
        
        // Get parameters
        std::string device = this->get_parameter("device").as_string();
        int width = this->get_parameter("width").as_int();
        int height = this->get_parameter("height").as_int();
        int framerate = this->get_parameter("framerate").as_int();
        std::string format = this->get_parameter("format").as_string();
        int jpeg_quality = this->get_parameter("jpeg_quality").as_int();
        int buffer_size = this->get_parameter("buffer_size").as_int();
        int max_buffers = this->get_parameter("max_buffers").as_int();
        bool use_nvmm = this->get_parameter("use_nvmm").as_bool();
        int io_mode = this->get_parameter("io_mode").as_int();
        publish_raw_ = this->get_parameter("publish_raw").as_bool();
        publish_compressed_ = this->get_parameter("publish_compressed").as_bool();
        frame_id_ = this->get_parameter("frame_id").as_string();
        std::string camera_info_url = this->get_parameter("camera_info_url").as_string();
        bool use_v4l2_timestamps = this->get_parameter("use_v4l2_timestamps").as_bool();
        
        // Configure processor
        V4L2ProcessorConfig config;
        config.device = device;
        config.width = width;
        config.height = height;
        config.framerate_num = framerate;
        config.framerate_den = 1;
        config.format = format;
        config.jpeg_quality = jpeg_quality;
        config.buffer_size = buffer_size;
        config.max_buffers = max_buffers;
        config.use_nvmm = use_nvmm;
        config.io_mode = io_mode;
        config.use_v4l2_timestamps = use_v4l2_timestamps;
        
        // Create processor
        processor_ = std::make_unique<V4L2Processor>(config);
        
        // Log hardware type
        switch (processor_->getHardwareType()) {
            case V4L2HardwareType::JETSON_NVMM:
                RCLCPP_INFO(this->get_logger(), "Using Jetson NVMM hardware acceleration");
                break;
            case V4L2HardwareType::JETSON_SOFTWARE:
                RCLCPP_INFO(this->get_logger(), "Using Jetson (software mode)");
                break;
            case V4L2HardwareType::SOFTWARE:
                RCLCPP_INFO(this->get_logger(), "Using software processing");
                break;
        }
        
        // Setup publishers
        if (publish_compressed_) {
            compressed_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
                "~/image_raw/compressed", 10);
        }
        
        if (publish_raw_) {
            image_transport::ImageTransport it(shared_from_this());
            raw_pub_ = it.advertise("~/image_raw", 10);
        }
        
        camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
            "~/camera_info", 10);
        
        // Initialize camera info manager
        camera_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, frame_id_);
        if (!camera_info_url.empty()) {
            camera_info_manager_->loadCameraInfo(camera_info_url);
            RCLCPP_INFO(this->get_logger(), "Loaded camera info from: %s", camera_info_url.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "No camera_info_url provided, using default camera info");
        }
        
        // Statistics timer
        stats_timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&GstImageProcessorNode::publishStatistics, this));
        
        // Set frame callback
        processor_->setFrameCallback(
            std::bind(&GstImageProcessorNode::frameCallback, this,
                     std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
        
        // Start processor
        if (!processor_->start()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to start V4L2 processor");
            rclcpp::shutdown();
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "GST Image Processor Node started");
        RCLCPP_INFO(this->get_logger(), "Device: %s", device.c_str());
        RCLCPP_INFO(this->get_logger(), "Resolution: %dx%d @ %d fps", width, height, framerate);
        RCLCPP_INFO(this->get_logger(), "Format: %s", format.c_str());
        RCLCPP_INFO(this->get_logger(), "Publishing: raw=%s, compressed=%s", 
                    publish_raw_ ? "true" : "false",
                    publish_compressed_ ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "NVMM: %s", use_nvmm ? "enabled" : "disabled");
    }
    
    ~GstImageProcessorNode() {
        if (processor_) {
            processor_->stop();
        }
    }
    
private:
    void frameCallback(const uint8_t* data, size_t size, const V4L2Processor::FrameTimestamp& timestamp) {
        // Convert V4L2 timestamp to ROS time
        rclcpp::Time ros_time;
        if (timestamp.capture_time_ns > 0) {
            // Use V4L2 buffer timestamp
            int64_t sec = timestamp.capture_time_ns / 1000000000LL;
            int64_t nsec = timestamp.capture_time_ns % 1000000000LL;
            ros_time = rclcpp::Time(sec, nsec);
        } else {
            // Fallback to current time
            ros_time = this->now();
        }
        
        // Publish compressed image
        if (publish_compressed_ && compressed_pub_->get_subscription_count() > 0) {
            auto compressed_msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
            compressed_msg->header.stamp = ros_time;
            compressed_msg->header.frame_id = frame_id_;
            compressed_msg->format = "jpeg";
            compressed_msg->data.assign(data, data + size);
            
            compressed_pub_->publish(std::move(compressed_msg));
        }
        
        // Publish raw image
        if (publish_raw_ && raw_pub_.getNumSubscribers() > 0) {
            std::vector<uint8_t> jpeg_data(data, data + size);
            cv::Mat compressed(1, jpeg_data.size(), CV_8UC1, jpeg_data.data());
            cv::Mat image = cv::imdecode(compressed, cv::IMREAD_COLOR);
            
            if (!image.empty()) {
                sensor_msgs::msg::Image::SharedPtr image_msg = 
                    cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
                image_msg->header.stamp = ros_time;
                image_msg->header.frame_id = frame_id_;
                
                raw_pub_.publish(image_msg);
            }
        }
        
        // Publish camera info
        if (camera_info_pub_->get_subscription_count() > 0) {
            auto camera_info_msg = camera_info_manager_->getCameraInfo();
            camera_info_msg.header.stamp = ros_time;
            camera_info_msg.header.frame_id = frame_id_;
            
            camera_info_pub_->publish(camera_info_msg);
        }
        
        frame_count_++;
    }
    
    void publishStatistics() {
        auto stats = processor_->getStatistics();
        RCLCPP_INFO(this->get_logger(), 
                   "Stats: Frames=%lu, Dropped=%lu, FPS=%.2f, Latency=%.2fms",
                   stats.frames_processed, stats.frames_dropped,
                   stats.average_fps, stats.average_latency_ms);
    }
    
    std::unique_ptr<V4L2Processor> processor_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
    image_transport::Publisher raw_pub_;
    rclcpp::TimerBase::SharedPtr stats_timer_;
    std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
    
    bool publish_raw_;
    bool publish_compressed_;
    std::string frame_id_;
    uint64_t frame_count_ = 0;
};

}  // namespace gst_image_processor

RCLCPP_COMPONENTS_REGISTER_NODE(gst_image_processor::GstImageProcessorNode)