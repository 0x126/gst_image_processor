#ifndef V4L2_PROCESSOR_IMPL_H
#define V4L2_PROCESSOR_IMPL_H

#include "v4l2_processor.h"
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <atomic>
#include <thread>
#include <mutex>
#include <chrono>
#include <deque>
#include <unordered_map>

class V4L2Processor::Impl {
public:
    explicit Impl(const V4L2ProcessorConfig& config);
    ~Impl();
    
    bool start();
    void stop();
    void setFrameCallback(FrameCallback callback);
    V4L2HardwareType getHardwareType() const { return hardware_type_; }
    Statistics getStatistics() const;
    
private:
    // GStreamer pipeline components
    GstElement* pipeline_ = nullptr;
    GstElement* source_ = nullptr;
    GstElement* capsfilter_ = nullptr;
    GstElement* converter_ = nullptr;
    GstElement* encoder_ = nullptr;
    GstElement* appsink_ = nullptr;
    
    // For Jetson NVMM optimization
    GstElement* nvvidconv_ = nullptr;
    GstElement* nvjpegenc_ = nullptr;

    // For timestamp probe
    GstElement* v4l2_identity_ = nullptr;

    // Configuration
    V4L2ProcessorConfig config_;
    V4L2HardwareType hardware_type_ = V4L2HardwareType::SOFTWARE;
    
    // Callback
    FrameCallback frame_callback_;
    std::mutex callback_mutex_;
    
    // Statistics
    std::atomic<uint64_t> frames_processed_{0};
    std::atomic<uint64_t> frames_dropped_{0};
    std::chrono::steady_clock::time_point start_time_;
    std::deque<std::chrono::steady_clock::time_point> frame_times_;
    std::deque<double> latencies_;
    mutable std::mutex stats_mutex_;
    
    // State
    std::atomic<bool> is_running_{false};
    
    // Helper methods
    bool createPipeline();
    bool createSoftwarePipeline();
    bool createJetsonNVMMPipeline();
    void detectHardware();
    static GstFlowReturn onNewSample(GstAppSink* sink, gpointer user_data);
    void processFrame(GstSample* sample);
    

    // Timestamp cache for probe
    std::unordered_map<GstBuffer*, int64_t> timestamp_cache_;
    std::mutex timestamp_cache_mutex_;

    // Probe setup and callback
    void setupV4L2TimestampProbe();
    static GstPadProbeReturn onV4L2Buffer(GstPad* pad, GstPadProbeInfo* info, gpointer user_data);
};

#endif // V4L2_PROCESSOR_IMPL_H