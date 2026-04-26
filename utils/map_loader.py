#读取.obj文件，或用文件夹内随机生成地图的方法生成一个地图，是工具类函数

import random
import open3d as o3d
import numpy as np
import yaml
from pathlib import Path
from typing import List, Tuple


def _rects_overlap(l1, r1, b1, t1, l2, r2, b2, t2, pad=0.0) -> bool:
    """判断两矩形（轴对齐）在 XY 平面上是否有重叠（允许 padding 扩展）。"""
    l1_adj, r1_adj, b1_adj, t1_adj = l1 - pad, r1 + pad, b1 - pad, t1 + pad
    l2_adj, r2_adj, b2_adj, t2_adj = l2 - pad, r2 + pad, b2 - pad, t2 + pad
    return not (r1_adj < l2_adj or r2_adj < l1_adj or t1_adj < b2_adj or t2_adj < b1_adj)


def generate_buildings(
    num_range: Tuple[int, int] = (180, 240),
    min_xy: float = -380.0,
    max_xy: float = 380.0,
    w_range: Tuple[float, float] = (12.0, 28.0),
    d_range: Tuple[float, float] = (12.0, 28.0),
    h_range: Tuple[float, float] = (45.0, 120.0),
    avoid_overlap: bool = True,
    padding: float = 2.5,
    max_attempts_per_building: int = 2000,
    config: dict = None,
) -> List[Tuple[Tuple[float, float, float], Tuple[float, float, float]]]:
    """生成随机城市建筑（支持避免 XY 平面重叠与可配置参数）。

    参数说明（均有合理默认值，兼容旧调用方式）:
    - num_range: 生成建筑数量的随机范围 (min, max)
    - min_xy/max_xy: 地图生成范围
    - w_range/d_range/h_range: 建筑宽、深、高的取值范围
    - avoid_overlap: 是否避免 XY 平面矩形重叠
    - padding: 避让缓冲（米），用于扩展占用矩形
    - max_attempts_per_building: 单栋建筑最大尝试位置次数
    - config: 可选的配置字典（若为 None，函数会尝试读取项目根目录下的 config.yaml 中的 map.buildings）

    返回: 列表，每项为 ((cx, cy, cz), (sx, sy, sz))
    """
    # 若未传入 config，尝试从项目根加载 config.yaml 中相关字段
    if config is None:
        try:
            cfg_path = Path(__file__).resolve().parents[1] / "config.yaml"
            if cfg_path.exists():
                with cfg_path.open("r", encoding="utf-8") as fh:
                    cfg = yaml.safe_load(fh) or {}
                    map_cfg = cfg.get("map", {})
                    bcfg = map_cfg.get("buildings", {})
                    if bcfg:
                        num_min = bcfg.get("count_min")
                        num_max = bcfg.get("count_max")
                        if num_min is not None and num_max is not None:
                            num_range = (int(num_min), int(num_max))
                        avoid_overlap = bool(bcfg.get("avoid_overlap", avoid_overlap))
                        padding = float(bcfg.get("padding", padding))
                        max_attempts_per_building = int(bcfg.get("max_attempts_per_building", max_attempts_per_building))
                        # 可选地覆盖大小范围
                        w_min = bcfg.get("width_min")
                        w_max = bcfg.get("width_max")
                        if w_min is not None and w_max is not None:
                            w_range = (float(w_min), float(w_max))
                        d_min = bcfg.get("depth_min")
                        d_max = bcfg.get("depth_max")
                        if d_min is not None and d_max is not None:
                            d_range = (float(d_min), float(d_max))
                        h_min = bcfg.get("height_min")
                        h_max = bcfg.get("height_max")
                        if h_min is not None and h_max is not None:
                            h_range = (float(h_min), float(h_max))
        except Exception:
            # 加载配置失败则使用默认参数
            pass

    buildings: List[Tuple[Tuple[float, float, float], Tuple[float, float, float]]] = []

    # 先添加预设的地标（保留区域，后续建筑不会覆盖）
    landmarks = [(-80, -80, 100), (80, -80, 110), (-80, 80, 95)]
    for x, y, h in landmarks:
        buildings.append(((x, y, h / 2.0), (18.0, 18.0, h)))

    target_num = random.randint(int(num_range[0]), int(num_range[1]))
    # 实际需要额外生成的数量（扣除地标）
    remaining = max(0, target_num - len(buildings))

    attempts_total = 0
    placed = 0
    for _ in range(remaining):
        placed_flag = False
        attempts = 0
        while attempts < max_attempts_per_building:
            attempts += 1
            attempts_total += 1
            x = random.uniform(min_xy, max_xy)
            y = random.uniform(min_xy, max_xy)
            # 避开基地附近（原点半径 50 米）
            if abs(x) < 50 and abs(y) < 50:
                continue
            # 避开特定禁区 (例如配置中的 (75,75) 半径 50)
            if (x - 75) ** 2 + (y - 75) ** 2 < 2500:
                continue

            w = random.uniform(float(w_range[0]), float(w_range[1]))
            d = random.uniform(float(d_range[0]), float(d_range[1]))
            h = random.uniform(float(h_range[0]), float(h_range[1]))

            left = x - w / 2.0
            right = x + w / 2.0
            bottom = y - d / 2.0
            top = y + d / 2.0

            if avoid_overlap:
                conflict = False
                for (ecenter, esize) in buildings:
                    ex, ey, _ = ecenter
                    ew, ed, _ = esize
                    el = ex - ew / 2.0
                    er = ex + ew / 2.0
                    eb = ey - ed / 2.0
                    et = ey + ed / 2.0
                    if _rects_overlap(left, right, bottom, top, el, er, eb, et, pad=padding):
                        conflict = True
                        break
                if conflict:
                    continue

            # 通过所有检测，加入列表
            center = (x, y, h / 2.0)
            size = (w, d, h)
            buildings.append((center, size))
            placed_flag = True
            placed += 1
            break

        if not placed_flag:
            # 未能在给定尝试次数内放置该建筑，跳过
            continue

    print(f"Generated {len(buildings)} buildings (requested {target_num}, placed {placed}, attempts {attempts_total}).")
    return buildings