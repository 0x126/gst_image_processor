#include "v4l2_processor_impl.h"
#include <gst/video/video.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <cstring>

V4L2Processor::Impl::Impl(const V4L2ProcessorConfig& config)
    : config_(config) {
    gst_init(nullptr, nullptr);
    detectHardware();
    calculateTSCOffset();
}

V4L2Processor::Impl::~Impl() {
    stop();
}

void V4L2Processor::Impl::detectHardware() {
    // Check if we're on Jetson platform
    std::ifstream tegra_release("/etc/nv_tegra_release");
    if (tegra_release.good()) {
        tegra_release.close();
        
        // Check if NVMM is available and requested
        if (config_.use_nvmm) {
            // Test if nvvidconv element is available
            GstElement* test = gst_element_factory_make("nvvidconv", nullptr);
            if (test) {
                gst_object_unref(test);
                hardware_type_ = V4L2HardwareType::JETSON_NVMM;
                std::cout << "Detected Jetson platform with NVMM support" << std::endl;
                return;
            }
        }
        hardware_type_ = V4L2HardwareType::JETSON_SOFTWARE;
        std::cout << "Detected Jetson platform (software mode)" << std::endl;
    } else {
        hardware_type_ = V4L2HardwareType::SOFTWARE;
        std::cout << "Using software processing" << std::endl;
    }
}

void V4L2Processor::Impl::calculateTSCOffset() {
    // Implementation based on ros2_v4l2_camera for L4T version >= 36
    #if defined(__arm__) || defined(__aarch64__)
    std::ifstream tegra_release("/etc/nv_tegra_release");
    if (tegra_release.good()) {
        tegra_release.close();
        
        // Use L4T version >= 36 method with ARM registers
        unsigned long raw_nsec, tsc_ns;
        unsigned long cycles, frq;
        struct timespec tp;
        
        // Read ARM timer frequency and cycle count
        asm volatile("mrs %0, cntfrq_el0" : "=r"(frq));
        asm volatile("mrs %0, cntvct_el0" : "=r"(cycles));
        
        // Get current CLOCK_MONOTONIC_RAW time
        clock_gettime(CLOCK_MONOTONIC_RAW, &tp);
        
        // Calculate TSC nanoseconds
        tsc_ns = (cycles * 100 / (frq / 10000)) * 1000;
        raw_nsec = tp.tv_sec * 1000000000 + tp.tv_nsec;
        
        // Calculate absolute difference
        tsc_offset_ = llabs(tsc_ns - raw_nsec);
        
        std::cout << "TSC offset calculated using ARM registers: " << tsc_offset_ << " ns" << std::endl;
        std::cout << "  Frequency: " << frq << " Hz" << std::endl;
        std::cout << "  Cycles: " << cycles << std::endl;
        std::cout << "  TSC ns: " << tsc_ns << std::endl;
        std::cout << "  Monotonic RAW ns: " << raw_nsec << std::endl;
        std::cout << "  Offset: " << tsc_offset_ << " ns (" << tsc_offset_ / 1000000000 << " seconds)" << std::endl;
    } else {
        tsc_offset_ = 0;
        std::cout << "Not running on Jetson platform, TSC offset set to 0" << std::endl;
    }
    #else
    tsc_offset_ = 0;
    std::cout << "Not ARM architecture, TSC offset set to 0" << std::endl;
    #endif
}

int64_t V4L2Processor::Impl::getTimeOffset() {
    // Get time offset between REALTIME and MONOTONIC_RAW
    timespec system_sample, monotonic_sample;
    clock_gettime(CLOCK_REALTIME, &system_sample);
    clock_gettime(CLOCK_MONOTONIC_RAW, &monotonic_sample);
    return (static_cast<int64_t>(system_sample.tv_sec * 1e9) - static_cast<int64_t>(monotonic_sample.tv_sec * 1e9)
            + static_cast<int64_t>(system_sample.tv_nsec) - static_cast<int64_t>(monotonic_sample.tv_nsec));
}

bool V4L2Processor::Impl::createPipeline() {
    if (hardware_type_ == V4L2HardwareType::JETSON_NVMM) {
        return createJetsonNVMMPipeline();
    } else {
        return createSoftwarePipeline();
    }
}

bool V4L2Processor::Impl::createSoftwarePipeline() {
    // Create pipeline
    pipeline_ = gst_pipeline_new("v4l2-pipeline");
    if (!pipeline_) {
        std::cerr << "Failed to create pipeline" << std::endl;
        return false;
    }
    
    // Create elements
    source_ = gst_element_factory_make("v4l2src", "source");
    capsfilter_ = gst_element_factory_make("capsfilter", "capsfilter");
    converter_ = gst_element_factory_make("videoconvert", "converter");
    encoder_ = gst_element_factory_make("jpegenc", "encoder");
    appsink_ = gst_element_factory_make("appsink", "sink");
    
    if (!source_ || !capsfilter_ || !converter_ || !encoder_ || !appsink_) {
        std::cerr << "Failed to create GStreamer elements" << std::endl;
        return false;
    }
    
    // Configure v4l2src
    g_object_set(source_,
                 "device", config_.device.c_str(),
                 "io-mode", config_.io_mode,
                 "do-timestamp", TRUE,
                 nullptr);
    
    // Set caps for v4l2src
    std::string caps_string = "video/x-raw"
                             ", width=" + std::to_string(config_.width) +
                             ", height=" + std::to_string(config_.height) +
                             ", framerate=" + std::to_string(config_.framerate_num) + 
                             "/" + std::to_string(config_.framerate_den) +
                             ", format=" + config_.format;
    
    GstCaps* caps = gst_caps_from_string(caps_string.c_str());
    g_object_set(capsfilter_, "caps", caps, nullptr);
    gst_caps_unref(caps);
    
    // Configure JPEG encoder
    g_object_set(encoder_,
                 "quality", config_.jpeg_quality,
                 nullptr);
    
    // Configure appsink
    g_object_set(appsink_,
                 "emit-signals", TRUE,
                 "sync", FALSE,
                 "max-buffers", config_.max_buffers,
                 "drop", TRUE,
                 nullptr);
    
    // Setup appsink callbacks
    GstAppSinkCallbacks callbacks = {};
    callbacks.new_sample = onNewSample;
    gst_app_sink_set_callbacks(GST_APP_SINK(appsink_), &callbacks, this, nullptr);
    
    // Add elements to pipeline
    gst_bin_add_many(GST_BIN(pipeline_),
                      source_, capsfilter_, converter_, encoder_, appsink_,
                      nullptr);
    
    // Link elements
    if (!gst_element_link_many(source_, capsfilter_, converter_, encoder_, appsink_, nullptr)) {
        std::cerr << "Failed to link elements" << std::endl;
        return false;
    }
    
    return true;
}

bool V4L2Processor::Impl::createJetsonNVMMPipeline() {
    // Create pipeline for Jetson with NVMM
    pipeline_ = gst_pipeline_new("v4l2-nvmm-pipeline");
    if (!pipeline_) {
        std::cerr << "Failed to create pipeline" << std::endl;
        return false;
    }
    
    // Create elements
    source_ = gst_element_factory_make("v4l2src", "source");
    capsfilter_ = gst_element_factory_make("capsfilter", "capsfilter");
    nvvidconv_ = gst_element_factory_make("nvvidconv", "nvvidconv");
    nvjpegenc_ = gst_element_factory_make("nvjpegenc", "nvjpegenc");
    appsink_ = gst_element_factory_make("appsink", "sink");
    
    if (!source_ || !capsfilter_ || !nvvidconv_ || !nvjpegenc_ || !appsink_) {
        std::cerr << "Failed to create GStreamer elements for Jetson NVMM" << std::endl;
        // Fallback to software pipeline
        hardware_type_ = V4L2HardwareType::JETSON_SOFTWARE;
        return createSoftwarePipeline();
    }
    
    // Configure v4l2src for NVMM
    g_object_set(source_,
                 "device", config_.device.c_str(),
                 "io-mode", 2,  // dmabuf for NVMM
                 "do-timestamp", TRUE,
                 nullptr);
    
    // Set caps for v4l2src with NVMM memory
    std::string caps_string = "video/x-raw(memory:NVMM)"
                             ", width=" + std::to_string(config_.width) +
                             ", height=" + std::to_string(config_.height) +
                             ", framerate=" + std::to_string(config_.framerate_num) + 
                             "/" + std::to_string(config_.framerate_den) +
                             ", format=" + config_.format;
    
    GstCaps* caps = gst_caps_from_string(caps_string.c_str());
    g_object_set(capsfilter_, "caps", caps, nullptr);
    gst_caps_unref(caps);
    
    // Configure nvvidconv if needed
    // nvvidconv handles format conversion and stays in NVMM memory
    
    // Configure nvjpegenc (hardware JPEG encoder)
    g_object_set(nvjpegenc_,
                 "quality", config_.jpeg_quality,
                 nullptr);
    
    // Configure appsink
    g_object_set(appsink_,
                 "emit-signals", TRUE,
                 "sync", FALSE,
                 "max-buffers", config_.max_buffers,
                 "drop", TRUE,
                 nullptr);
    
    // Setup appsink callbacks
    GstAppSinkCallbacks callbacks = {};
    callbacks.new_sample = onNewSample;
    gst_app_sink_set_callbacks(GST_APP_SINK(appsink_), &callbacks, this, nullptr);
    
    // Add elements to pipeline
    gst_bin_add_many(GST_BIN(pipeline_),
                      source_, capsfilter_, nvvidconv_, nvjpegenc_, appsink_,
                      nullptr);
    
    // Link elements
    if (!gst_element_link_many(source_, capsfilter_, nvvidconv_, nvjpegenc_, appsink_, nullptr)) {
        std::cerr << "Failed to link NVMM elements" << std::endl;
        // Try without NVMM
        hardware_type_ = V4L2HardwareType::JETSON_SOFTWARE;
        gst_object_unref(pipeline_);
        pipeline_ = nullptr;
        return createSoftwarePipeline();
    }
    
    std::cout << "Successfully created Jetson NVMM pipeline" << std::endl;
    return true;
}

bool V4L2Processor::Impl::start() {
    if (is_running_) {
        return true;
    }
    
    if (!createPipeline()) {
        return false;
    }
    
    // Start pipeline
    GstStateChangeReturn ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
        std::cerr << "Failed to start pipeline" << std::endl;
        gst_object_unref(pipeline_);
        pipeline_ = nullptr;
        return false;
    }
    
    is_running_ = true;
    start_time_ = std::chrono::steady_clock::now();
    
    return true;
}

void V4L2Processor::Impl::stop() {
    if (!is_running_ || !pipeline_) {
        return;
    }
    
    is_running_ = false;
    
    // Stop pipeline
    gst_element_set_state(pipeline_, GST_STATE_NULL);
    gst_object_unref(pipeline_);
    pipeline_ = nullptr;
}

void V4L2Processor::Impl::setFrameCallback(FrameCallback callback) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    frame_callback_ = callback;
}

GstFlowReturn V4L2Processor::Impl::onNewSample(GstAppSink* sink, gpointer user_data) {
    auto* impl = static_cast<Impl*>(user_data);
    
    GstSample* sample = gst_app_sink_pull_sample(sink);
    if (sample) {
        impl->processFrame(sample);
        gst_sample_unref(sample);
    }
    
    return GST_FLOW_OK;
}

void V4L2Processor::Impl::processFrame(GstSample* sample) {
    auto now = std::chrono::steady_clock::now();
    
    GstBuffer* buffer = gst_sample_get_buffer(sample);
    if (!buffer) {
        frames_dropped_++;
        return;
    }
    
    // Get buffer timestamp
    GstClockTime pts = GST_BUFFER_PTS(buffer);
    int64_t capture_time_ns = 0;
    
    if (GST_CLOCK_TIME_IS_VALID(pts)) {
        // GStreamer PTS is in nanoseconds from pipeline start
        // For V4L2, this should be MONOTONIC time
        
        if (config_.use_v4l2_timestamps) {
            // Convert MONOTONIC to REALTIME
            // PTS is already in MONOTONIC domain, just add offset to get REALTIME
            capture_time_ns = pts + getTimeOffset();
            
            // Note: TSC offset correction might not be needed here since
            // v4l2src with do-timestamp=true should already provide correct MONOTONIC timestamps
        } else {
            // Use current time
            struct timespec ts;
            clock_gettime(CLOCK_REALTIME, &ts);
            capture_time_ns = ts.tv_sec * 1000000000LL + ts.tv_nsec;
        }
    } else {
        // Use current time if PTS is not valid
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        capture_time_ns = ts.tv_sec * 1000000000LL + ts.tv_nsec;
    }
    
    // Get system time for comparison/debugging
    struct timespec system_ts;
    clock_gettime(CLOCK_REALTIME, &system_ts);
    int64_t system_time_ns = system_ts.tv_sec * 1000000000LL + system_ts.tv_nsec;
    
    // Map buffer
    GstMapInfo map;
    if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) {
        frames_dropped_++;
        return;
    }
    
    // Create timestamp
    FrameTimestamp timestamp;
    timestamp.capture_time_ns = capture_time_ns;
    timestamp.system_time_ns = system_time_ns;
    timestamp.sequence = frames_processed_;
    
    // Call callback
    {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        if (frame_callback_) {
            frame_callback_(map.data, map.size, timestamp);
        }
    }
    
    gst_buffer_unmap(buffer, &map);
    
    // Update statistics
    frames_processed_++;
    
    {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        frame_times_.push_back(now);
        if (frame_times_.size() > 100) {
            frame_times_.pop_front();
        }
        
        double latency_ms = std::chrono::duration<double, std::milli>(now - std::chrono::steady_clock::now()).count();
        latencies_.push_back(latency_ms);
        if (latencies_.size() > 100) {
            latencies_.pop_front();
        }
    }
}

V4L2Processor::Statistics V4L2Processor::Impl::getStatistics() const {
    Statistics stats;
    stats.frames_processed = frames_processed_;
    stats.frames_dropped = frames_dropped_;
    
    std::lock_guard<std::mutex> lock(stats_mutex_);
    
    // Calculate FPS
    if (frame_times_.size() > 1) {
        auto duration = std::chrono::duration<double>(
            frame_times_.back() - frame_times_.front()).count();
        stats.average_fps = (frame_times_.size() - 1) / duration;
    } else {
        stats.average_fps = 0.0;
    }
    
    // Calculate average latency
    if (!latencies_.empty()) {
        double sum = 0.0;
        for (double latency : latencies_) {
            sum += latency;
        }
        stats.average_latency_ms = sum / latencies_.size();
    } else {
        stats.average_latency_ms = 0.0;
    }
    
    return stats;
}