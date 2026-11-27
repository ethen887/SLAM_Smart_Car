"""Host navigation tests. Run from any directory with Python and GCC installed."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[3]
ra = root / 'Renesas/TinySLAM/3.LSLidar/src'
slam_sources = [ra/'Slam/map_core.c', ra/'Slam/slam_scan_accum.c', ra/'Slam/slam_matcher.c',
                ra/'Slam/slam_pose_graph.c', ra/'Slam/slam_engine.c']
with tempfile.TemporaryDirectory(prefix='navigation-tests-') as temporary:
    for name, includes, sources in (
        ('navigation', [root/'tools/tests/navigation', root/'tools/tests/real_mapping',
                        ra/'Application', ra/'Navigation',
                        ra/'Data_Process', ra/'Device', ra, ra/'Slam'],
         [root/'tools/tests/navigation/test_navigation.c', ra/'Navigation/nav_path.c',
          ra/'Navigation/nav_path_smoother.c', ra/'Navigation/nav_velocity_profile.c',
          ra/'Navigation/nav_tracker.c',
          ra/'Application/app_navigation.c', ra/'Application/app_navigation_plan.c',
          ra/'Application/app_global_plan.c', ra/'Application/app_buildmap.c',
          ra/'Navigation/nav_navfn.c', ra/'Navigation/nav_costmap.c', ra/'Data_Process/nav_usb_protocol.c',
          *slam_sources]),
        ('velocity_plan', [ra/'Navigation', ra/'Slam', ra],
         [root/'tools/tests/navigation/test_velocity_plan.c', ra/'Navigation/nav_path.c',
          ra/'Navigation/nav_costmap.c', ra/'Navigation/nav_path_smoother.c',
          ra/'Navigation/nav_velocity_profile.c', ra/'Navigation/nav_tracker.c']),
        ('usb_serial', [root/'tools/tests/usb_serial', ra/'Data_Process'],
         [root/'tools/tests/usb_serial/test_usb_serial.c']),
        ('real_mapping', [root/'tools/tests/real_mapping', root/'tools/tests/navigation', ra/'Application', ra/'Data_Process', ra/'Device', ra,
                          ra/'Slam'],
         [root/'tools/tests/real_mapping/test_real_mapping.c', ra/'Application/app_buildmap.c',
          *slam_sources]),
        ('motion', [root/'STM32_F103/App', root/'tools/tests/host_comm'],
         [root/'tools/tests/navigation/test_motion.c']),
        ('host_comm', [root/'STM32_F103/App', root/'tools/tests/host_comm'],
         [root/'tools/tests/host_comm/test_host_comm.c', root/'STM32_F103/App/app_host_comm.c',
          root/'STM32_F103/App/host_protocol.c']),
        ('split_frames', [root/'STM32_F103/App', ra/'Data_Process', ra/'Device'],
         [root/'tools/tests/real_mapping/test_split_frames.c', ra/'Device/link_stm32.c',
          ra/'Device/lidar_n10.c', root/'STM32_F103/App/host_protocol.c']),
    ):
        output = Path(temporary) / (name + '.exe')
        subprocess.run(['gcc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                        *['-I'+str(p) for p in includes], *map(str, sources), '-o', str(output)], check=True)
        subprocess.run([str(output)], check=True)
