import launch

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():

    tracking_base_frame = DeclareLaunchArgument('tracking_base_frame', 
                                                 default_value="kinect_rgb_camera_link",
                                                 description="e.g. camera frame")
    tracking_marker_frame = DeclareLaunchArgument('tracking_marker_frame',
                                                  default_value="tag_2")
    robot_base_frame = DeclareLaunchArgument('robot_base_frame',
                                             default_value="base_link")
    robot_effector_frame = DeclareLaunchArgument('robot_effector_frame',
                                                 default_value="gripper_r_base")
    calibration_type = DeclareLaunchArgument('calibration_type',
                                             default_value="eye-on-base",
                                             description="Options are eye-in-hand or eye-on-base")

    calibration_node = Node(
            package='hand_eye_calibration',
            executable='hand_eye_calibration',
            name='hand_eye_calibration',
            output='screen',
            parameters=[
                {'tracking_base_frame': launch.substitutions.LaunchConfiguration('tracking_base_frame')},
                {'tracking_marker_frame': launch.substitutions.LaunchConfiguration('tracking_marker_frame')},
                {'robot_base_frame': launch.substitutions.LaunchConfiguration('robot_base_frame')},
                {'robot_effector_frame': launch.substitutions.LaunchConfiguration('robot_effector_frame')},
                {'calibration_type': launch.substitutions.LaunchConfiguration('calibration_type')},
            ]
    )    

    ll = list()
    ll.append(tracking_base_frame)
    ll.append(tracking_marker_frame)
    ll.append(robot_base_frame)
    ll.append(robot_effector_frame)
    ll.append(calibration_type)
    ll.append(calibration_node)

    return LaunchDescription(ll)