from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='orbit_bot_base',
            executable='base_driver',
            name='base_driver',
            output='screen',
            parameters=[
                {'wheel_radius': 0.04},
                {'track_width': 0.26},
                {'cpr': 2320.0},
                {'max_rpm': 333.0},
                {'max_linear_speed': 1.4},
                {'max_angular_speed': 1.0},
                {'left_pwm_multiplier': 0.9435},
                {'right_pwm_multiplier': 1.0},
                {'serial_port': '/dev/ttyCH341USB0'},
                {'baud_rate': 115200}
            ]
        ),
        # You can add teleop_twist_keyboard here if you want it to launch together, 
        # but it's usually better to run it in a separate terminal.
    ])
