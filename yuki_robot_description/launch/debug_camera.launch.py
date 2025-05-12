from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch file for visualizing camera images and rectified images
    """
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Image view nodes for debugging
    raw_image_view = Node(
        package='image_view',
        executable='image_view',
        name='raw_image_view',
        remappings=[
            ('image', '/depth_camera/image_raw'),
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    rect_image_view = Node(
        package='image_view',
        executable='image_view',
        name='rect_image_view',
        remappings=[
            ('image', '/depth_camera/image_rect'),
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Create and populate the launch description
    ld = LaunchDescription()
    
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(raw_image_view)
    ld.add_action(rect_image_view)
    
    return ld
