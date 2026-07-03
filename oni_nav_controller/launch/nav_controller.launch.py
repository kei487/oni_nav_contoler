from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
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

    return LaunchDescription([
        SetEnvironmentVariable('LD_LIBRARY_PATH', ld_library_path),
        Node(
            package='oni_nav_controller',
            executable='nav_controller_node',
            name='nav_controller_node',
            output='screen',
            parameters=[config_path],
        ),
    ])
