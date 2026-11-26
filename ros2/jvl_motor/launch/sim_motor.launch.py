"""Run the JVL driver against a loopback MAC400 simulator."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import ExecutableInPackage


def launch_motor(context):
    # rospy resolves private names under the node's fully qualified ROS name.
    namespace = context.launch_configurations.get('ros_namespace', '').strip('/').replace('/', '.')
    prefix = namespace + '.' if namespace else ''
    port = LaunchConfiguration('motor_modbus_port')
    params_file = LaunchConfiguration('params_file').perform(context)
    params = [params_file] if params_file else []
    wait = ExecutableInPackage(package='phyto_sim_utils', executable='wait_for_socket')
    return [
        Node(package='jvl_motor', executable='mac400_sim_node.py', name='mac400_sim',
             parameters=params + [{prefix + 'mac400_sim.address': '127.0.0.1',
                                   prefix + 'mac400_sim.port': port}],
             on_exit=[Shutdown(reason='MAC400 simulator exited')]),
        Node(package='jvl_motor', executable='jvl_motor_node.py', name='motor',
             prefix=[wait, ' ', port],
             parameters=params + [{prefix + 'motor.address': '127.0.0.1',
                                   prefix + 'motor.port': port,
                                   prefix + 'motor.refresh_rate': LaunchConfiguration('refresh_rate')}],
             on_exit=[Shutdown(reason='JVL motor driver exited')]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('motor_modbus_port', default_value='1502'),
        DeclareLaunchArgument('refresh_rate', default_value='1'),
        DeclareLaunchArgument('params_file', default_value=''),
        OpaqueFunction(function=launch_motor),
    ])
