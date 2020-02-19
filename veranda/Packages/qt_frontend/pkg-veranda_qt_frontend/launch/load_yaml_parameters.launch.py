import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

from pathlib import Path
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    parameter_file_veranda = Path(get_package_share_directory('veranda_qt_frontend'), 'config', 'veranda_qt_frontend.yaml')
    assert parameter_file_veranda.is_file()

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='veranda_qt_frontend',
            node_executable='veranda',
            output='screen',
            parameters=[parameter_file_veranda]
        )
    ])