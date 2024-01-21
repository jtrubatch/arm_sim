import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnExecutionComplete

def generate_launch_description():
    
    arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ur_mtc'), 'launch'),
            '/spawn_arm.launch.py'])
    )

    controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ur_mtc'), 'launch'),
            '/controllers.launch.py'])
    )

    setup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ur_mtc'), 'launch'),
            '/mtc_setup2.launch.py'])
    )

    controller_event = TimerAction(
        period=3.0,
        actions=[controllers_launch]
    )


    setup_event = TimerAction(
        period=6.0,
        actions=[setup_launch]
    )

    return LaunchDescription([
        arm_launch,
        controller_event,
        setup_event,
    ])