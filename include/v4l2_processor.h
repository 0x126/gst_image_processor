#ifndef V4L2_PROCESSOR_H
#define V4L2_PROCESSOR_H

#include <memory>
#include <functional>
#include <cstdint>
#include <cstddef>
#include <string>

enum class V4L2HardwareType {
    SOFTWARE,           // Software processing
    JETSON_NVMM,       // Jetson with NVMM (hardware acceleration)
    JETSON_SOFTWARE    // Jetson without NVMM (software fallback)
};

struct V4L2ProcessorConfig {
    // V4L2 source parameters
    std::string device = "/dev/video0";
    int width = 2880;
    int height = 1860;
    int framerate_num = 20;
    int framerate_den = 1;
    std::string format = "UYVY";  // V4L2 format
    
    // Processing parameters
    int jpeg_quality = 90;
    int buffer_size = 8388608;  // 8MB
    int max_buffers = 3;
    
    // Hardware acceleration
    bool use_nvmm = false;  // Use NVIDIA Memory Management (Jetson)
    int io_mode = 0;        // 0: mmap, 2: dmabuf (for NVMM)
    
    // Output format
    std::string output_format = "BGR";  // BGR, RGB, NV12
    
    // Timestamp handling
    bool use_v4l2_timestamps = true;
};

class V4L2Processor {
public:
    struct FrameTimestamp {
        int64_t capture_time_ns;    // Capture time in nanoseconds (from V4L2 buffer)
        int64_t system_time_ns;     // System time when frame was received
        uint32_t sequence;           // Frame sequence number
    };
    
    using FrameCallback = std::function<void(const uint8_t* data, size_t size, const FrameTimestamp& timestamp)>;
    
    V4L2Processor(const V4L2ProcessorConfig& config);
    ~V4L2Processor();
    
    // Disable copy
    V4L2Processor(const V4L2Processor&) = delete;
    V4L2Processor& operator=(const V4L2Processor&) = delete;
    
    // Start/stop processing
    bool start();
    void stop();
    
    // Set callback for processed frames
    void setFrameCallback(FrameCallback callback);
    
    // Get detected hardware type
    V4L2HardwareType getHardwareType() const;
    
    // Statistics
    struct Statistics {
        uint64_t frames_processed;
        uint64_t frames_dropped;
        double average_fps;
        double average_latency_ms;
    };
    Statistics getStatistics() const;
    
private:
    class Impl;
    std::unique_ptr<Impl> pImpl;
};

#endif // V4L2_PROCESSOR_H