import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='veranda_qt_frontend',
            node_executable='veranda',
            output='screen',
            parameters=[{'auto_load_asset': 'no','json_asset_path': '/path/to/asset/my_asset.json', 'auto_start_sim': 'yes'}]
        )
    ])