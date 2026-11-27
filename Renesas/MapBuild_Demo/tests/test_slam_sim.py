"""SLAM 仿真闭环对比与实车 N10 数据流回放单元测试。"""
import copy
import csv
import json
import math
import unittest
from pathlib import Path
import numpy as np

from run_demo import build
from simulation import Simulation, SlamGrid, defaults


ROOT = Path(__file__).resolve().parents[1]


def _run_scenario(scene, lib, use_slam):
    cfg = copy.deepcopy(scene)
    cfg['slam'] = use_slam
    cfg['slam_loop_closure'] = True
    sim = Simulation(cfg, lib)
    for _ in range(4000):
        if sim.finished:
            break
        sim.step()
    errs = [math.hypot(h['estimate'][0] - h['real'][0], h['estimate'][1] - h['real'][1])
            for h in sim.history]
    rmse = math.sqrt(sum(e * e for e in errs) / len(errs))
    max_err = max(errs)
    stats = sim.history[-1]['slam_stats']
    sim.grid.close()
    return rmse, max_err, stats


class TestSlamSimulation(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.lib = build()
        cls.slam_lib = cls.lib.with_name(
            'libslamcore.dll' if cls.lib.suffix == '.dll' else 'libslamcore.so'
        )

    def test_default_scene_ideal_odom_no_stairstep(self):
        """默认 5x5m 场景 + 理想里程计下无边界误剪枝与误回环，并验证 0.04m 真机分辨率精度。"""
        scene = defaults()
        slam_rmse, slam_max, stats = _run_scenario(scene, self.lib, use_slam=True)

        # 0.05m 零边距栅格按 floorf 量化存在每轴 0.025m（合 ~0.035m）半格中心偏移，且开放路径不应误回环
        self.assertLess(slam_rmse, 0.045)
        self.assertLess(slam_max, 0.06)
        self.assertIsNotNone(stats)
        self.assertEqual(stats['loop_closures'], 0)

        # 与真机一致的 0.04m 分辨率 + 留边原点下应达到亚格级高精度
        scene_004 = defaults()
        scene_004['map'].update({'width': 150, 'height': 150, 'resolution': 0.04,
                                 'origin_x': -3.0, 'origin_y': -3.0})
        rmse_004, max_004, stats_004 = _run_scenario(scene_004, self.lib, use_slam=True)
        self.assertLess(rmse_004, 0.02)
        self.assertLess(max_004, 0.03)
        self.assertGreater(stats_004['last_score'], 0.90)
        self.assertEqual(stats_004['loop_closures'], 0)

    def test_complex_loop_drift_correction(self):
        """在复杂路径 + 大里程计漂移下，SLAM 显著压低轨迹误差并完成回环。"""
        scene = json.loads((ROOT / 'Demo' / '一次复杂路径的尝试.json').read_text(encoding='utf8'))
        scene['closed'] = True
        scene['odom'] = {'scale': 1.08, 'heading_bias': 0.05, 'noise': 0.005}
        scene['map'].update({'width': 150, 'height': 150, 'resolution': 0.04,
                             'origin_x': -3.0, 'origin_y': -3.0})

        odom_rmse, odom_max, _ = _run_scenario(scene, self.lib, use_slam=False)
        slam_rmse, slam_max, stats = _run_scenario(scene, self.lib, use_slam=True)

        self.assertGreater(odom_rmse, 0.5)
        self.assertLess(slam_rmse, odom_rmse * 0.15)
        self.assertLess(slam_max, 0.18)
        self.assertIsNotNone(stats)
        self.assertGreaterEqual(stats['loop_closures'], 1)

    def test_real_n10_capture_replay(self):
        """使用实车采集的 N10 16 点包流验证 360° 拼帧与真实噪声下的匹配稳定性。"""
        csv_path = ROOT / 'out' / 'lidar_capture' / '20261123_182112_dy9py9ki' / 'lidar_points.csv'
        if not csv_path.exists():
            self.skipTest('未找到实车雷达录制文件')

        map_cfg = defaults()['map']
        map_cfg.update({'width': 300, 'height': 300, 'resolution': 0.04,
                        'origin_x': -6.0, 'origin_y': -6.0})
        slam = SlamGrid(self.slam_lib, map_cfg, min_range=0.08, max_range=8.0, loop_closure=True)

        # 按 packet_seq 分组喂入前 800 个 N10 数据包（约 30+ 圈完整扫描）
        packets = {}
        with csv_path.open('r', encoding='utf8') as f:
            reader = csv.DictReader(f)
            for row in reader:
                seq = int(row['packet_seq'])
                if len(packets) >= 800 and seq not in packets:
                    break
                if int(row['valid_distance']) == 0:
                    continue
                ang_rad = -math.radians(float(row['angle_deg_cw']))
                dist_m = float(row['distance_mm']) * 0.001
                packets.setdefault(seq, []).append((ang_rad, dist_m, 1))

        produced = 0
        poses = []
        for seq in sorted(packets):
            rays = packets[seq]
            if not rays:
                continue
            ok, pose = slam.push_packet([0.0, 0.0, 0.0], rays)
            if ok:
                produced += 1
                poses.append(pose)

        st = slam.stats()
        grid = slam.snapshot()
        slam.close()

        self.assertGreaterEqual(produced, 15)
        self.assertGreater(st['frontend_accepted'], 10)
        self.assertGreater(int(grid['seen'].sum()), 1000)
        # 静态或微动实车扫描不应发生剧烈发散跳变
        max_jump = max(math.hypot(p[0], p[1]) for p in poses)
        self.assertLess(max_jump, 0.5)


if __name__ == '__main__':
    unittest.main()
