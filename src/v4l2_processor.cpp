#include "v4l2_processor.h"
#include "v4l2_processor_impl.h"

V4L2Processor::V4L2Processor(const V4L2ProcessorConfig& config)
    : pImpl(std::make_unique<Impl>(config)) {
}

V4L2Processor::~V4L2Processor() {
    stop();
}

bool V4L2Processor::start() {
    return pImpl->start();
}

void V4L2Processor::stop() {
    pImpl->stop();
}

void V4L2Processor::setFrameCallback(FrameCallback callback) {
    pImpl->setFrameCallback(callback);
}

V4L2HardwareType V4L2Processor::getHardwareType() const {
    return pImpl->getHardwareType();
}

V4L2Processor::Statistics V4L2Processor::getStatistics() const {
    return pImpl->getStatistics();
}