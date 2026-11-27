"""生成纯里程计 vs 纯 C99 SLAM（前端匹配 + 回环图优化 + 地图重绘）的量化与图像对比报告。"""
import copy
import csv
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np

from run_demo import build
from simulation import Simulation, SlamGrid, defaults


def run_sim(scene, lib, use_slam):
    cfg = copy.deepcopy(scene)
    cfg['slam'] = use_slam
    cfg['slam_loop_closure'] = True
    sim = Simulation(cfg, lib)
    for _ in range(5000):
        if sim.finished:
            break
        sim.step()
    pos_errs = [math.hypot(h['estimate'][0] - h['real'][0], h['estimate'][1] - h['real'][1])
                for h in sim.history]
    ang_errs = [abs((h['estimate'][2] - h['real'][2] + math.pi) % (2 * math.pi) - math.pi)
                for h in sim.history]
    metrics = {
        'frames': len(sim.history),
        'pos_rmse_m': math.sqrt(sum(e * e for e in pos_errs) / len(pos_errs)),
        'pos_max_m': max(pos_errs),
        'ang_rmse_deg': math.degrees(math.sqrt(sum(e * e for e in ang_errs) / len(ang_errs))),
        'stats': sim.history[-1]['slam_stats'],
        'history': sim.history,
        'grid': sim.history[-1]['grid'],
    }
    sim.grid.close()
    return metrics


def grid_to_img(grid):
    return np.where(grid['seen'], 1.0 - 1.0 / (1.0 + np.exp(-grid['log'])), 0.5)


def main():
    print('正在构建 C 核心并运行默认场景（理想里程计）回归检查...', flush=True)
    lib = build()
    slam_lib = lib.with_name('libslamcore.dll' if lib.suffix == '.dll' else 'libslamcore.so')
    default_scene = defaults()
    default_slam = run_sim(default_scene, lib, use_slam=True)
    print(f"[默认场景 SLAM] Pos RMSE = {default_slam['pos_rmse_m']:.4f} m, "
          f"Max = {default_slam['pos_max_m']:.4f} m, "
          f"末帧得分 = {default_slam['stats']['last_score']:.3f}", flush=True)

    scene = json.loads((ROOT / 'Demo' / '一次复杂路径的尝试.json').read_text(encoding='utf8'))
    scene['closed'] = True
    scene['odom'] = {'scale': 1.08, 'heading_bias': 0.05, 'noise': 0.005}
    scene['map'].update({'width': 150, 'height': 150, 'resolution': 0.04,
                         'origin_x': -3.0, 'origin_y': -3.0})

    print('正在运行复杂闭环路径仿真对比（共约 1250 帧，请稍候约 10~20 秒）...', flush=True)
    odom_res = run_sim(scene, lib, use_slam=False)
    slam_res = run_sim(scene, lib, use_slam=True)

    print('=== 复杂闭环路径 + 大里程计误差 (scale=1.08, heading_bias=0.05 rad/s) ===')
    print(f"[纯里程计] Pos RMSE = {odom_res['pos_rmse_m']:.4f} m, "
          f"Max = {odom_res['pos_max_m']:.4f} m, Ang RMSE = {odom_res['ang_rmse_deg']:.2f} deg")
    print(f"[纯C SLAM] Pos RMSE = {slam_res['pos_rmse_m']:.4f} m, "
          f"Max = {slam_res['pos_max_m']:.4f} m, Ang RMSE = {slam_res['ang_rmse_deg']:.2f} deg")
    print(f"[SLAM统计] {slam_res['stats']}")

    # 绘制三联对比图
    plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False
    cfg = scene['map']
    extent = [cfg['origin_x'], cfg['origin_x'] + cfg['width'] * cfg['resolution'],
              cfg['origin_y'], cfg['origin_y'] + cfg['height'] * cfg['resolution']]

    fig, axes = plt.subplots(1, 3, figsize=(15, 5), dpi=130)
    axes[0].set_title('真实环境与轨迹')
    axes[0].add_patch(Rectangle((-2.5, -2.5), 5, 5, fill=False, edgecolor='black', linewidth=2))
    for o in scene['obstacles']:
        axes[0].add_patch(Rectangle((o['x'], o['y']), o['w'], o['h'], facecolor='#555555'))
    rx = [h['real'][0] for h in slam_res['history']]
    ry = [h['real'][1] for h in slam_res['history']]
    axes[0].plot(rx, ry, color='#1f77b4', linewidth=1.6, label='真值轨迹')
    axes[0].set(xlim=(-2.8, 2.8), ylim=(-2.8, 2.8), aspect='equal')
    axes[0].legend(loc='upper right')

    axes[1].set_title(f"纯里程计建图（重影严重）\nRMSE={odom_res['pos_rmse_m']:.3f}m")
    axes[1].imshow(grid_to_img(odom_res['grid']), origin='lower', extent=extent, cmap='gray', vmin=0, vmax=1)
    axes[1].plot(rx, ry, color='#2ca02c', linestyle=':', linewidth=1.2, label='真值')
    axes[1].plot([h['estimate'][0] for h in odom_res['history']],
                 [h['estimate'][1] for h in odom_res['history']],
                 color='#d62728', linewidth=1.4, label='里程计估计')
    axes[1].set(xlim=(-2.8, 2.8), ylim=(-2.8, 2.8), aspect='equal')
    axes[1].legend(loc='upper right')

    st = slam_res['stats']
    axes[2].set_title(f"纯C SLAM（CSM+GN+回环重绘）\nRMSE={slam_res['pos_rmse_m']:.3f}m, 闭环={st['loop_closures']}次")
    axes[2].imshow(grid_to_img(slam_res['grid']), origin='lower', extent=extent, cmap='gray', vmin=0, vmax=1)
    axes[2].plot(rx, ry, color='#2ca02c', linestyle=':', linewidth=1.2, label='真值')
    axes[2].plot([h['estimate'][0] for h in slam_res['history']],
                 [h['estimate'][1] for h in slam_res['history']],
                 color='#1f77b4', linewidth=1.4, label='SLAM 估计')
    axes[2].set(xlim=(-2.8, 2.8), ylim=(-2.8, 2.8), aspect='equal')
    axes[2].legend(loc='upper right')

    out_dir = ROOT / 'out'
    out_dir.mkdir(exist_ok=True)
    sim_png = out_dir / 'slam_compare_sim.png'
    fig.tight_layout()
    fig.savefig(sim_png)
    plt.close(fig)
    print(f'仿真对比图已保存：{sim_png}')

    # 实车 N10 数据流回放验证
    csv_path = ROOT / 'out' / 'lidar_capture' / '20261123_182112_dy9py9ki' / 'lidar_points.csv'
    if csv_path.exists():
        print('正在回放实车 N10 雷达数据流（4341 包）...', flush=True)
        map_cfg = defaults()['map']
        map_cfg.update({'width': 300, 'height': 300, 'resolution': 0.04,
                        'origin_x': -6.0, 'origin_y': -6.0})
        slam = SlamGrid(slam_lib, map_cfg, min_range=0.08, max_range=8.0, loop_closure=True)
        packets = {}
        with csv_path.open('r', encoding='utf8') as f:
            for row in csv.DictReader(f):
                seq = int(row['packet_seq'])
                if int(row['valid_distance']) == 0:
                    continue
                ang_rad = -math.radians(float(row['angle_deg_cw']))
                dist_m = float(row['distance_mm']) * 0.001
                packets.setdefault(seq, []).append((ang_rad, dist_m, 1))

        produced = 0
        poses = []
        for seq in sorted(packets):
            ok, pose = slam.push_packet([0.0, 0.0, 0.0], packets[seq])
            if ok:
                produced += 1
                poses.append(pose)
        real_st = slam.stats()
        real_grid = slam.snapshot()
        slam.close()
        max_drift = max((math.hypot(p[0], p[1]) for p in poses), default=0.0)
        print(f'\n=== 实车 N10 数据流回放（{len(packets)} 包 -> {produced} 圈完整扫描）===')
        print(f'[实车统计] {real_st}, max_pos_drift={max_drift:.4f} m')

        fig2, ax = plt.subplots(figsize=(6, 6), dpi=130)
        ax.set_title(f'实车 N10 雷达流式拼帧 + SLAM 建图（{produced} 圈）')
        ax.imshow(grid_to_img(real_grid), origin='lower', extent=[-6, 6, -6, 6],
                  cmap='gray', vmin=0, vmax=1)
        ax.plot([p[0] for p in poses], [p[1] for p in poses], color='#1f77b4', linewidth=1.2)
        ax.set(xlim=(-5, 5), ylim=(-5, 5), aspect='equal', xlabel='X (m)', ylabel='Y (m)')
        real_png = out_dir / 'slam_real_n10_replay.png'
        fig2.tight_layout()
        fig2.savefig(real_png)
        plt.close(fig2)
        print(f'实车回放图已保存：{real_png}')


if __name__ == '__main__':
    main()
