import os
from datetime import datetime
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def prompt_for_recording(context, *args, **kwargs):
    """Prompt user for rosbag recording at launch time."""
    actions = []

    record_enabled = LaunchConfiguration('record').perform(context)

    if record_enabled.lower() == 'ask':
        print("\n" + "="*50)
        print("  ROSBAG RECORDING")
        print("="*50)
        response = input("Start rosbag recording? [y/N]: ").strip().lower()
        should_record = response in ['y', 'yes']
    else:
        should_record = record_enabled.lower() in ['true', '1', 'yes', 'y']

    if should_record:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        bag_dir = LaunchConfiguration('bag_dir').perform(context)
        bag_path = os.path.join(bag_dir, f"vn110_recording_{timestamp}")
        os.makedirs(bag_dir, exist_ok=True)

        print(f"\nRecording to: {bag_path}")
        print("Topics:")
        print("  - /vn110/imu")
        print("  - /vn110/euler")
        print("  - /vn110/status/*")
        print("  - /fmu/out/sensor_combined")
        print("  - /fmu/out/vehicle_attitude")
        print("  - /fmu/out/vehicle_attitude_euler")
        print("  - /fmu/out/airspeed_validated")
        print("  - /dead_reckoning/*")
        print("")

        record_cmd = ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record',
                '-o', bag_path,
                '-s', 'mcap',
                '/vn110/imu',
                '/vn110/euler',
                '/vn110/status/gyro',
                '/vn110/status/accel',
                '/vn110/status/mag',
                '/vn110/status/pressure',
                '/vn110/status/any_saturation',
                '/fmu/out/sensor_combined',
                '/fmu/out/vehicle_attitude',
                '/fmu/out/vehicle_attitude_euler',
                '/fmu/out/airspeed_validated',
                '/dead_reckoning/vn110_position',
                '/dead_reckoning/px4_position',
                '/dead_reckoning/drift',
                '/dead_reckoning/velocity',
            ],
            output='screen'
        )
        actions.append(TimerAction(period=2.0, actions=[record_cmd]))
    else:
        print("\nRosbag recording: DISABLED\n")

    return actions


def generate_launch_description():
    pkg_share = get_package_share_directory('vn110_bridge')
    default_params = os.path.join(pkg_share, 'config', 'vn110_params.yaml')

    return LaunchDescription([
        # Recording arguments
        DeclareLaunchArgument('record',  default_value='ask'),
        DeclareLaunchArgument('bag_dir', default_value=os.path.expanduser('~/rosbags')),

        # Optional: override the params file path
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='Path to VN110 parameter YAML file'),

        # Connection override (takes priority over YAML)
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),

        # VN110 Bridge Node - YAML first, then launch arg overrides
        Node(
            package='vn110_bridge',
            executable='vn110_bridge_node',
            name='vn110_bridge',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {'port': LaunchConfiguration('port')},
            ],
        ),

        OpaqueFunction(function=prompt_for_recording),
    ])
