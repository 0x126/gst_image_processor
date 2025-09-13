#include "v4l2_processor_impl.h"
#include <gst/video/video.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <cstring>
#include <cstdlib>

V4L2Processor::Impl::Impl(const V4L2ProcessorConfig& config)
    : config_(config) {
    gst_init(nullptr, nullptr);
    detectHardware();
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
    v4l2_identity_ = gst_element_factory_make("identity", "v4l2probe");
    capsfilter_ = gst_element_factory_make("capsfilter", "capsfilter");
    converter_ = gst_element_factory_make("videoconvert", "converter");
    encoder_ = gst_element_factory_make("jpegenc", "encoder");
    appsink_ = gst_element_factory_make("appsink", "sink");

    if (!source_ || !v4l2_identity_ || !capsfilter_ || !converter_ || !encoder_ || !appsink_) {
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
                      source_, v4l2_identity_, capsfilter_, converter_, encoder_, appsink_,
                      nullptr);

    // Link elements
    if (!gst_element_link_many(source_, v4l2_identity_, capsfilter_, converter_, encoder_, appsink_, nullptr)) {
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
    v4l2_identity_ = gst_element_factory_make("identity", "v4l2probe");
    capsfilter_ = gst_element_factory_make("capsfilter", "capsfilter");
    nvvidconv_ = gst_element_factory_make("nvvidconv", "nvvidconv");
    nvjpegenc_ = gst_element_factory_make("nvjpegenc", "nvjpegenc");
    appsink_ = gst_element_factory_make("appsink", "sink");

    if (!source_ || !v4l2_identity_ || !capsfilter_ || !nvvidconv_ || !nvjpegenc_ || !appsink_) {
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
    std::string caps_string = "video/x-raw"
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
                      source_, v4l2_identity_, capsfilter_, nvvidconv_, nvjpegenc_, appsink_,
                      nullptr);

    // Link elements
    if (!gst_element_link_many(source_, v4l2_identity_, capsfilter_, nvvidconv_, nvjpegenc_, appsink_, nullptr)) {
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

    std::cout << "Creating pipeline..." << std::endl;
    if (!createPipeline()) {
        std::cerr << "Failed to create pipeline" << std::endl;
        return false;
    }

    // Setup probe to capture timestamps right after v4l2src
    setupV4L2TimestampProbe();

    std::cout << "Starting pipeline..." << std::endl;
    // Start pipeline
    GstStateChangeReturn ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
        std::cerr << "Failed to start pipeline" << std::endl;
        
        // Get error details
        GstBus* bus = gst_element_get_bus(pipeline_);
        GstMessage* msg = gst_bus_pop_filtered(bus, GST_MESSAGE_ERROR);
        if (msg != nullptr) {
            GError* err;
            gchar* debug_info;
            gst_message_parse_error(msg, &err, &debug_info);
            std::cerr << "Error: " << err->message << std::endl;
            std::cerr << "Debug info: " << debug_info << std::endl;
            g_clear_error(&err);
            g_free(debug_info);
            gst_message_unref(msg);
        }
        gst_object_unref(bus);
        
        gst_object_unref(pipeline_);
        pipeline_ = nullptr;
        return false;
    }
    
    // Wait for pipeline to reach playing state
    GstState state;
    GstState pending;
    ret = gst_element_get_state(pipeline_, &state, &pending, 5 * GST_SECOND);
    if (ret == GST_STATE_CHANGE_FAILURE) {
        std::cerr << "Pipeline failed to reach PLAYING state" << std::endl;
        gst_element_set_state(pipeline_, GST_STATE_NULL);
        gst_object_unref(pipeline_);
        pipeline_ = nullptr;
        return false;
    }
    
    std::cout << "Pipeline started successfully" << std::endl;
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
    } else {
        std::cerr << "No sample received from appsink" << std::endl;
    }
    
    return GST_FLOW_OK;
}

void V4L2Processor::Impl::processFrame(GstSample* sample) {
    static bool first_frame = true;
    if (first_frame) {
        std::cout << "First frame received at appsink!" << std::endl;
        first_frame = false;
    }

    auto now = std::chrono::steady_clock::now();

    GstBuffer* buffer = gst_sample_get_buffer(sample);
    if (!buffer) {
        frames_dropped_++;
        return;
    }

    // Get current time for comparison
    struct timespec system_ts;
    clock_gettime(CLOCK_REALTIME, &system_ts);
    int64_t system_time_ns = system_ts.tv_sec * 1000000000LL + system_ts.tv_nsec;

    // Try to get timestamp from cache (captured at v4l2src output)
    int64_t capture_time_ns = system_time_ns;  // Default to current time

    GstClockTime pts = GST_BUFFER_PTS(buffer);
    if (config_.use_v4l2_timestamps && GST_CLOCK_TIME_IS_VALID(pts)) {
        std::lock_guard<std::mutex> lock(timestamp_cache_mutex_);
        auto it = timestamp_cache_.find(pts);
        if (it != timestamp_cache_.end()) {
            capture_time_ns = it->second;
            timestamp_cache_.erase(it);  // Remove used entry

            // Debug: show latency for first few frames
            if (frames_processed_ < 5) {
                double latency_ms = (system_time_ns - capture_time_ns) / 1000000.0;
                std::cout << "Frame " << frames_processed_
                          << ": V4L2 capture to JPEG output latency = "
                          << latency_ms << " ms" << std::endl;
            }
        }
    }
    
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

void V4L2Processor::Impl::setupV4L2TimestampProbe() {
    if (!v4l2_identity_) {
        return;  // identity element not created (shouldn't happen)
    }

    GstPad* srcpad = gst_element_get_static_pad(v4l2_identity_, "src");
    if (!srcpad) {
        std::cerr << "Failed to get srcpad from v4l2_identity" << std::endl;
        return;
    }

    // Add probe to capture timestamps right after v4l2src
    gst_pad_add_probe(srcpad, GST_PAD_PROBE_TYPE_BUFFER,
                      onV4L2Buffer, this, nullptr);
    gst_object_unref(srcpad);
}

GstPadProbeReturn V4L2Processor::Impl::onV4L2Buffer(GstPad* /*pad*/, GstPadProbeInfo* info, gpointer user_data) {
    auto* impl = static_cast<Impl*>(user_data);
    GstBuffer* buffer = GST_PAD_PROBE_INFO_BUFFER(info);

    if (!buffer) {
        return GST_PAD_PROBE_OK;
    }

    // Get PTS for this buffer
    GstClockTime pts = GST_BUFFER_PTS(buffer);
    if (!GST_CLOCK_TIME_IS_VALID(pts)) {
        return GST_PAD_PROBE_OK;
    }

    // Capture current time when frame arrives from V4L2
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    int64_t realtime_ns = ts.tv_sec * 1000000000LL + ts.tv_nsec;

    // Store in cache using PTS as key
    {
        std::lock_guard<std::mutex> lock(impl->timestamp_cache_mutex_);
        impl->timestamp_cache_[pts] = realtime_ns;

        // Clean up old entries if cache gets too large
        if (impl->timestamp_cache_.size() > 30) {
            // Remove oldest entries (simple cleanup - in production, use LRU)
            auto it = impl->timestamp_cache_.begin();
            std::advance(it, 10);
            impl->timestamp_cache_.erase(impl->timestamp_cache_.begin(), it);
        }
    }

    return GST_PAD_PROBE_OK;
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