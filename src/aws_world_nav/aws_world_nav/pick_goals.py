#!/usr/bin/env python3
"""从保存的地图(pgmap+yaml)中自动挑选可导航目标点。

流程: 解析 pgm/yaml -> 提取空闲栅格 -> 腐蚀(机器人半径) -> 最大连通域 ->
     以出生点附近为起点做最远点采样 -> 输出世界坐标。

用法:
    python3 pick_goals.py maps/house.yaml --n 3 --spawn 0,0 --format flat
输出:
    flat: "x1,y1,0.0,x2,y2,0.0,..."  (供 send_goals 的 goals 参数)
    json: [[x1,y1,0.0], ...]
仅依赖 numpy / pyyaml。
"""

import argparse
import json
import math
import os
import sys
from collections import deque

import numpy as np
import yaml


def load_map(yaml_path):
    with open(yaml_path, 'r') as f:
        meta = yaml.safe_load(f)
    pgm_path = meta['image']
    if not os.path.isabs(pgm_path):
        pgm_path = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), pgm_path)
    with open(pgm_path, 'rb') as f:
        assert f.readline().strip() == b'P5', '仅支持 P5 PGM'
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
        w, h = map(int, line.split())
        maxval = int(f.readline())
        assert maxval == 255
        data = np.frombuffer(f.read(), dtype=np.uint8).reshape(h, w)
    return meta, data


def erode(mask, radius):
    """方形结构元腐蚀（近似圆形）。"""
    r = int(radius)
    out = mask.copy()
    for dy in range(-r, r + 1):
        for dx in range(-r, r + 1):
            if dx * dx + dy * dy > r * r:
                continue
            shifted = np.roll(np.roll(mask, dy, axis=0), dx, axis=1)
            out &= shifted
    return out


def largest_component(mask):
    h, w = mask.shape
    label = np.zeros((h, w), dtype=np.int32)
    cur = 0
    best_id, best_size = 0, 0
    for sy in range(h):
        for sx in range(w):
            if mask[sy, sx] and label[sy, sx] == 0:
                cur += 1
                size = 0
                q = deque([(sy, sx)])
                label[sy, sx] = cur
                while q:
                    y, x = q.popleft()
                    size += 1
                    for ny, nx in ((y+1, x), (y-1, x), (y, x+1), (y, x-1)):
                        if 0 <= ny < h and 0 <= nx < w and mask[ny, nx] and label[ny, nx] == 0:
                            label[ny, nx] = cur
                            q.append((ny, nx))
                if size > best_size:
                    best_id, best_size = cur, size
    return label == best_id


def world_to_cell(meta, data, wx, wy):
    res = float(meta['resolution'])
    ox, oy = float(meta['origin'][0]), float(meta['origin'][1])
    h = data.shape[0]
    col = int((wx - ox) / res)
    row = int(h - 1 - (wy - oy) / res)
    return row, col


def cell_to_world(meta, data, row, col):
    res = float(meta['resolution'])
    ox, oy = float(meta['origin'][0]), float(meta['origin'][1])
    h = data.shape[0]
    return (ox + (col + 0.5) * res, oy + (h - 1 - row + 0.5) * res)


def farthest_point_sampling(free, start_rc, n):
    """从 start_rc 开始迭代选取欧氏距离最远的空闲点。"""
    ys, xs = np.nonzero(free)
    pts = np.stack([ys, xs], axis=1).astype(np.float64)
    chosen = []
    d = np.linalg.norm(pts - np.array(start_rc, dtype=np.float64), axis=1)
    for _ in range(n):
        i = int(np.argmax(d))
        chosen.append((int(pts[i, 0]), int(pts[i, 1])))
        d = np.minimum(d, np.linalg.norm(pts - pts[i], axis=1))
    return chosen


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('map_yaml')
    ap.add_argument('--n', type=int, default=3)
    ap.add_argument('--spawn', default='0,0', help='出生点 x,y（世界坐标）')
    ap.add_argument('--robot-radius', type=float, default=0.3,
                    help='腐蚀半径 m，保证目标点离障碍足够远')
    ap.add_argument('--format', choices=['flat', 'json'], default='flat')
    args = ap.parse_args()

    meta, data = load_map(args.map_yaml)
    # 空闲: 灰度接近 254 的像素
    free = data >= 250
    if not free.any():
        print('ERROR: 地图中没有空闲区域', file=sys.stderr)
        sys.exit(1)

    res = float(meta['resolution'])
    free = erode(free, args.robot_radius / res)
    free = largest_component(free)
    if not free.any():
        print('ERROR: 腐蚀后没有足够空闲区域', file=sys.stderr)
        sys.exit(1)

    sx, sy = map(float, args.spawn.split(','))
    start_rc = world_to_cell(meta, data, sx, sy)
    # 若出生点不在最大连通域内，改用该连通域质心出发
    if not (0 <= start_rc[0] < data.shape[0] and 0 <= start_rc[1] < data.shape[1]) \
            or not free[start_rc[0], start_rc[1]]:
        ys, xs = np.nonzero(free)
        start_rc = (int(ys.mean()), int(xs.mean()))

    cells = farthest_point_sampling(free, start_rc, args.n)
    goals = [list(cell_to_world(meta, data, r, c)) + [0.0] for r, c in cells]

    if args.format == 'json':
        print(json.dumps(goals))
    else:
        flat = []
        for g in goals:
            flat += [f'{g[0]:.3f}', f'{g[1]:.3f}', f'{g[2]:.3f}']
        print(','.join(flat))


if __name__ == '__main__':
    main()
