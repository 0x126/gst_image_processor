from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='/dev/video0',
        description='V4L2 device path'
    )
    
    width_arg = DeclareLaunchArgument(
        'width',
        default_value='2880',
        description='Image width'
    )
    
    height_arg = DeclareLaunchArgument(
        'height',
        default_value='1860',
        description='Image height'
    )
    
    framerate_arg = DeclareLaunchArgument(
        'framerate',
        default_value='30',
        description='Frame rate'
    )
    
    format_arg = DeclareLaunchArgument(
        'format',
        default_value='UYVY',
        description='V4L2 format (UYVY, YUYV, etc.)'
    )
    
    use_nvmm_arg = DeclareLaunchArgument(
        'use_nvmm',
        default_value='false',
        description='Use NVIDIA Memory Management (Jetson only)'
    )
    
    io_mode_arg = DeclareLaunchArgument(
        'io_mode',
        default_value='4',
        description='V4L2 IO mode (0: mmap, 2: dmabuf for NVMM)'
    )
    
    jpeg_quality_arg = DeclareLaunchArgument(
        'jpeg_quality',
        default_value='90',
        description='JPEG compression quality (1-100)'
    )
    
    publish_raw_arg = DeclareLaunchArgument(
        'publish_raw',
        default_value='false',
        description='Publish raw images'
    )
    
    publish_compressed_arg = DeclareLaunchArgument(
        'publish_compressed',
        default_value='true',
        description='Publish compressed images'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='camera',
        description='Frame ID for the camera'
    )
    
    camera_info_url_arg = DeclareLaunchArgument(
        'camera_info_url',
        default_value='',
        description='URL for camera calibration file'
    )
    
    use_v4l2_timestamps_arg = DeclareLaunchArgument(
        'use_v4l2_timestamps',
        default_value='true',
        description='Use V4L2 buffer timestamps'
    )
    
    # Node
    gst_image_processor_node = Node(
        package='gst_image_processor',
        executable='gst_image_processor_node',
        name='gst_image_processor',
        output='screen',
        parameters=[{
            'device': LaunchConfiguration('device'),
            'width': LaunchConfiguration('width'),
            'height': LaunchConfiguration('height'),
            'framerate': LaunchConfiguration('framerate'),
            'format': LaunchConfiguration('format'),
            'use_nvmm': LaunchConfiguration('use_nvmm'),
            'io_mode': LaunchConfiguration('io_mode'),
            'jpeg_quality': LaunchConfiguration('jpeg_quality'),
            'publish_raw': LaunchConfiguration('publish_raw'),
            'publish_compressed': LaunchConfiguration('publish_compressed'),
            'frame_id': LaunchConfiguration('frame_id'),
            'camera_info_url': LaunchConfiguration('camera_info_url'),
            'use_v4l2_timestamps': LaunchConfiguration('use_v4l2_timestamps'),
        }]
    )
    
    return LaunchDescription([
        device_arg,
        width_arg,
        height_arg,
        framerate_arg,
        format_arg,
        use_nvmm_arg,
        io_mode_arg,
        jpeg_quality_arg,
        publish_raw_arg,
        publish_compressed_arg,
        frame_id_arg,
        camera_info_url_arg,
        use_v4l2_timestamps_arg,
        gst_image_processor_node
    ])