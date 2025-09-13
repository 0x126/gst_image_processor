# GST Image Processor

ROS2 node for processing V4L2 camera images using GStreamer with Jetson optimization support.

## Features

- Direct V4L2 camera capture using GStreamer
- Jetson NVMM hardware acceleration support
- Automatic TSC timestamp correction for Jetson platforms
- Compatible with `rtp_image_processor` interface
- Publishes both raw and compressed images
- Camera info support

## Hardware Support

The node automatically detects and uses the best available hardware:

1. **Jetson with NVMM**: Hardware-accelerated processing using NVIDIA Memory Management
2. **Jetson Software**: Software processing on Jetson platforms
3. **Generic x86/ARM**: Software processing using standard GStreamer elements

## Installation

```bash
# Install dependencies
sudo apt-get install -y \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-libav

# Build
colcon build --packages-select gst_image_processor
```

## Usage

### Basic Usage (Software Mode)

```bash
# Python launch file
ros2 launch gst_image_processor gst_image_processor.launch.py

# XML launch file
ros2 launch gst_image_processor gst_image_processor.launch.xml
```

### Custom Parameters

```bash
ros2 run gst_image_processor gst_image_processor_node --ros-args \
    -p device:=/dev/video0 \
    -p width:=2880 \
    -p height:=1860 \
    -p framerate:=20 \
    -p format:=UYVY \
    -p use_nvmm:=true \
    -p jpeg_quality:=90
```

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `device` | string | `/dev/video0` | V4L2 device path |
| `width` | int | `2880` | Image width |
| `height` | int | `1860` | Image height |
| `framerate` | int | `30` | Frame rate |
| `format` | string | `UYVY` | V4L2 format (UYVY, YUYV, etc.) |
| `use_nvmm` | bool | `false` | Enable NVIDIA Memory Management (Jetson only) |
| `io_mode` | int | `2` | V4L2 IO mode (0: auto, 2: mmap, 4: dmabuf) |
| `jpeg_quality` | int | `90` | JPEG compression quality (1-100) |
| `publish_raw` | bool | `false` | Publish raw images |
| `publish_compressed` | bool | `true` | Publish compressed images |
| `frame_id` | string | `camera` | Frame ID for camera |
| `camera_info_url` | string | `""` | URL for camera calibration file |
| `use_v4l2_timestamps` | bool | `true` | Use V4L2 buffer timestamps |

## Topics

### Published Topics

- `image_raw/compressed` (sensor_msgs/CompressedImage): JPEG compressed images
- `image_raw` (sensor_msgs/Image): Raw BGR images (if enabled)
- `camera_info` (sensor_msgs/CameraInfo): Camera calibration information

## GStreamer Pipeline Examples

The node internally constructs GStreamer pipelines similar to:

### Software Mode
```bash
gst-launch-1.0 v4l2src device=/dev/video0 do-timestamp=true ! \
    'video/x-raw, width=2880, height=1860, framerate=20/1, format=UYVY' ! \
    videoconvert ! jpegenc quality=90 ! appsink
```

### Jetson NVMM Mode
```bash
gst-launch-1.0 v4l2src io-mode=2 device=/dev/video0 ! \
    'video/x-raw(memory:NVMM), width=2880, height=1860, framerate=20/1, format=UYVY' ! \
    nvvidconv ! nvjpegenc quality=90 ! appsink
```

## Timestamp Handling

The node includes automatic TSC (Time Stamp Counter) offset correction for Jetson platforms, ensuring accurate timestamps when using V4L2 buffer timestamps. This is compatible with the approach used in `ros2_v4l2_camera`.

## Performance Tips

1. **For Jetson platforms**: Enable NVMM with `use_nvmm:=true` and `io_mode:=2`
2. **Adjust buffer settings**: Modify `buffer_size` and `max_buffers` based on your requirements
3. **JPEG quality**: Lower quality values reduce bandwidth but decrease image quality
4. **Disable raw publishing**: Set `publish_raw:=false` if not needed to reduce CPU usage

## Troubleshooting

1. **NVMM not working**: Ensure you have the necessary NVIDIA drivers and GStreamer plugins installed
2. **Timestamp issues**: Check TSC offset is being correctly calculated (see logs)
3. **High latency**: Try reducing `max_buffers` or disabling sync in the pipeline