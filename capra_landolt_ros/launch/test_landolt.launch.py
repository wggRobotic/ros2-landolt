import os
import unittest
import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
from ament_index_python.packages import get_package_share_directory

def generate_test_description():
    pkg_share = get_package_share_directory('capra_landolt_ros')
    sample_data_path = os.path.join(pkg_share, 'test', 'sample') + '/'

    # Deine Landolt Node starten
    landolt_node = launch_ros.actions.Node(
        package='capra_landolt_ros',
        executable='landolt_node',
        name='landolt_detector',
        parameters=[{
            'camera_reading': '/capra/camera_3d/rgb/image_raw',
            'queue_size': 5
        }],
        output='screen'
    )

    # Die C++ GTest Executable aufrufen
    test_executable = launch_ros.actions.Node(
        package='capra_landolt_ros',
        executable='landolt_test_exe',
        parameters=[{'datapath': sample_data_path}],
        output='screen'
    )

    return launch.LaunchDescription([
        landolt_node,
        test_executable,
        launch_testing.actions.ReadyToTest()
    ]), {'test_executable': test_executable, 'landolt_node': landolt_node}


class TestLandoltLaufzeit(unittest.TestCase):
    def test_gtest_executes(self, proc_info, test_executable):
        # Dieser Pre-Shutdown-Test zwingt das Framework, auf die Executable zu warten!
        # Er blockiert hier maximal 15 Sekunden oder bricht ab, sobald die Node fertig ist.
        proc_info.assertWaitForShutdown(process=test_executable, timeout=15.0)


@launch_testing.post_shutdown_test()
class TestResultValidation(unittest.TestCase):
    def test_node_codes(self, proc_info, test_executable):
        # Wenn wir hier ankommen, ist die Node regulär fertig. Jetzt prüfen wir den Exit-Code.
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[launch_testing.asserts.EXIT_OK],
            process=test_executable
        )