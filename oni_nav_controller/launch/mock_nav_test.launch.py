from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('oni_nav_controller')
    config_path = os.path.join(pkg_share, 'config', 'nav_controller.yaml')

    acados_install = os.environ.get(
        'ACADOS_INSTALL_DIR',
        os.path.abspath(os.path.join(pkg_share, '..', '..', '..', 'deps', 'acados_install')),
    )
    acados_lib = os.path.join(acados_install, 'lib')
    ld_library_path = acados_lib
    if 'LD_LIBRARY_PATH' in os.environ:
        ld_library_path = f"{acados_lib}:{os.environ['LD_LIBRARY_PATH']}"

    mode_arg = DeclareLaunchArgument('mode', default_value='chase')

    return LaunchDescription([
        SetEnvironmentVariable('LD_LIBRARY_PATH', ld_library_path),
        mode_arg,
        Node(
            package='oni_nav_controller',
            executable='nav_controller_node',
            name='nav_controller_node',
            output='screen',
            parameters=[config_path],
        ),
        Node(
            package='oni_nav_controller',
            executable='mock_nav_inputs.py',
            name='mock_nav_inputs',
            output='screen',
            parameters=[{'mode': LaunchConfiguration('mode')}],
        ),
    ])
