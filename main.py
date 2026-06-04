import math
import random
import time
import msvcrt
from dataclasses import fields
from pathlib import Path

import yaml
import os

from Controller.collision_handler import detect_collisions, resolve_collisions, is_offensive_type
from drones.offensive.attack_drone import AttackDrone
from drones.offensive.tank_drone import TankDrone
from drones.offensive.anti_radiation_drone import AntiRadiationDrone
from drones.offensive.ew_drone import EWDrone
from drones.defensive.scout_drone import ScoutDrone
from drones.defensive.interceptor_drone import InterceptorDrone
from drones.defensive.anti_ew_drone import AntiEWDrone
from drones.factory import (
    create_drone_team, create_attack_drone, create_tank_drone,
    create_anti_radiation_drone, create_ew_drone, create_anti_ew_drone,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
    create_interceptor_drone, create_scout_drone,
)
from Controller.single_control import chase_target, chase_point, move_drone
from utils.logger import get_logger
from utils.geometry import distance                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          
from base.base_manager import BaseManager

LOGGER = get_logger(__name__)

# 全局变量，用于存储 Open3D 显示对象，如果导入失败则为 None
Open3DDisplay = None
OPEN3D_IMPORT_ERROR = None

try:
    from Visual.open3d_display import Open3DDisplay
except Exception as exc:
    OPEN3D_IMPORT_ERROR = exc
    LOGGER.warning("Open3D visualization not available: %s", exc)


def load_config(path: Path) -> dict:
    """加载 YAML 配置文件"""
    with path.open("r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def _is_offensive(drone):
    """ 判断无人机是否是进攻方"""
    return is_offensive_type(drone.drone_type)


def _find_nearest_opponent(drone, candidates):
    """ 寻找最近的敌人, 传入代表自己的实例(drone), 与可能要追逐的“候选”的敌方无人机的列表candidates"""
    best = None
    best_dist = float("inf")
    for other in candidates:
        if other is drone or not other.is_alive():
            continue
        dist = sum((p - q) ** 2 for p, q in zip(drone.position, other.position))
        if dist < best_dist:
            best_dist = dist
            best = other
    #初始化最佳敌人为None
    #初始化最佳距离为无穷大
    #遍历候选无人机列表candidates
        #跳过自己与已坠毁的无人机
        #遍历所有其它无人机单位, 找到与自己距离最短的那个, 最终将其标记为best
    #返回与自己距离最短的无人机实例(best)
    return best


_ASSIGNMENT_CONFIG = None
_OFFENSIVE_AVOID_SETTINGS = {
    "enabled": True,
    "low_altitude_threshold": 30.0,
    "corridor_cells": 1,
    "sample_count": 8,
}

# ---------------------------------------------------------------------------
# 电子战 (EW) 干扰相关工具
# ---------------------------------------------------------------------------

def _get_ew_drones(drones: list):
    """返回所有存活且干扰激活的电子战无人机。"""
    return [
        d for d in drones
        if isinstance(d, EWDrone) and d.jamming_active and d.is_alive()
    ]


def _apply_ew_jamming(drones: list, ew_drones: list) -> None:
    """每帧根据 EW 无人机的位置标记防守方是否被干扰。

    被干扰的防守方：_ew_jammed = True，不参与任务分配且路径规划慢速。
    AntiEWDrone 不受 EW 干扰影响。
    """
    for d in drones:
        # 只标记防守方（非进攻方），且 AntiEWDrone 免疫
        if is_offensive_type(d.drone_type) or isinstance(d, AntiEWDrone):
            d._ew_jammed = False
            continue
        if not d.is_alive():
            d._ew_jammed = False
            continue
        jammed = False
        for ew in ew_drones:
            if ew.is_in_jamming_range(d.position):
                jammed = True
                break
        d._ew_jammed = jammed


# 在文件顶部（例如 _ASSIGNMENT_CONFIG 后面）添加全局变量
_LAST_ANTI_EW_SPAWN_TIME = 0.0
_ANTI_EW_SPAWN_COOLDOWN = 2.0   # 冷却时间（秒）
_MAX_ANTI_EW_RATIO = 1.5        # AntiEW 数量 / EW 数量 的上限

def _spawn_anti_ew_if_needed(config: dict, drones: list, base_positions: list | None):
    global _LAST_ANTI_EW_SPAWN_TIME
    import time
    now = time.time()
    if now - _LAST_ANTI_EW_SPAWN_TIME < _ANTI_EW_SPAWN_COOLDOWN:
        return

    ew_drones = _get_ew_drones(drones)
    if not ew_drones:
        return

    existing_anti_ew = [d for d in drones if isinstance(d, AntiEWDrone) and d.is_alive()]
    # 比例限制：AntiEW 数量不应超过 EW 数量的 _MAX_ANTI_EW_RATIO 倍
    if len(existing_anti_ew) >= len(ew_drones) * _MAX_ANTI_EW_RATIO:
        return

    needed = len(ew_drones) - len(existing_anti_ew)
    if needed <= 0:
        return
    # 每次最多生成 1 架，避免瞬间爆发
    needed = min(needed, 1)
    for _ in range(needed):
        name = f"AntiEW-{random.randint(100, 999)}"
        # 选择一个基地进行生成：若提供基地列表，则选择距离首个 EW 最近的基地
        spawn_base = None
        if base_positions:
            try:
                # 以第一个 EW 为参考，寻找最近基地
                ref = ew_drones[0]
                best = None
                best_dist = float("inf")
                for bp in base_positions:
                    dx = ref.position[0] - bp[0]
                    dy = ref.position[1] - bp[1]
                    dz = ref.position[2] - (bp[2] if len(bp) > 2 else 0.0)
                    d = (dx * dx + dy * dy + dz * dz) ** 0.5
                    if d < best_dist:
                        best_dist = d
                        best = bp
                spawn_base = best
            except Exception:
                spawn_base = None
        if spawn_base is None:
            spawn_pos = config.get("base", {}).get("position", [0.0, 0.0, 50.0])
            spawn_pos = [spawn_pos[0], spawn_pos[1], 50.0]
        else:
            spawn_pos = [spawn_base[0], spawn_base[1], 50.0]
        create_anti_ew_drone(name, spawn_pos, config, drones)
        LOGGER.info("Spawned AntiEW drone: %s at %s", name, spawn_pos)
    _LAST_ANTI_EW_SPAWN_TIME = now

def _check_anti_ew_disruption(ew_drones: list, anti_ew_drones: list) -> None:
    """检查反电子战无人机是否靠近 EW 无人机，靠近则永久关闭其干扰。"""
    for aew in anti_ew_drones:
        if not aew.is_alive():
            continue
        for ew in ew_drones:
            if not ew.jamming_active:
                continue
            dist = sum((p - q) ** 2 for p, q in zip(aew.position, ew.position)) ** 0.5
            if dist <= aew.disruption_radius:
                ew.disable_jamming()
                LOGGER.info(
                    "AntiEW %s disabled EW %s jamming (dist=%.1f)",
                    "AntiEW %s disabled EW %s jamming (dist=%.1f)",                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       
                    "AntiEW %s disabled EW %s jamming (dist=%.1f)",
                    aew.name, ew.name, dist,
                )


# ====== 以下为原有 ARD 感知屏蔽工具 ======

def _get_jammers(drones: list):
    """返回所有存活且屏蔽已初始化的反辐射无人机。"""
    return [
        d for d in drones
        if isinstance(d, AntiRadiationDrone) and d.jamming_active
    ]


def _is_visible_to_defense(drone, base_positions: list | None, jammers: list) -> bool:
    """进攻方无人机是否对防守方可见（不在屏蔽扇区内）。

    对于多基地场景，ARD 在创建时已固化其绑定的基地，is_in_jammed_sector
    会使用该绑定基地作为原点；因此此处无需传入统一的 base_position。
    """
    if not jammers:
        return True
    for jammer in jammers:
        # 让 jammer 自己判断（使用其内部绑定的基地位置）
        if jammer.is_in_jammed_sector(drone.position, None):
            return False
    return True


def _get_jammed_directions(jammers: list) -> list:
    """返回所有活跃屏蔽方向（弧度），去重。"""
    directions = []
    for j in jammers:
        if j.jamming_direction is not None:
            rounded = round(j.jamming_direction, 2)
            if rounded not in directions:
                directions.append(rounded)
    return directions


def _dispatch_defenders_to_jammed_directions(
    defensive: list,
    jammers: list,
    base_positions: list | None,
    map_grid=None,
) -> None:
    """向每个活跃屏蔽方向派遣 1 侦察 + 1 拦截（从无分配目标的防守方中选取）。"""
    if not jammers:
        return
    unassigned = [
        d for d in defensive
        if d.is_alive() and getattr(d, "_assigned_target", None) is None
    ]
    if not unassigned:
        return
    scouts = [d for d in unassigned if isinstance(d, ScoutDrone)]
    interceptors = [d for d in unassigned if isinstance(d, InterceptorDrone)]
    # 为每个活跃 ARD 派遣防守单元，ARD 在初始化时已绑定其基地位置（_base_position）
    for jammer in jammers:
        if not getattr(jammer, "jamming_active", False):
            continue
        direction = jammer.jamming_direction
        base_pos = getattr(jammer, "_base_position", None)
        if base_pos is None and base_positions:
            # 退化：选择第一个基地作为参考
            base_pos = base_positions[0]
        if base_pos is None or direction is None:
            continue
        target_x = base_pos[0] + 300.0 * math.cos(direction)
        target_y = base_pos[1] + 300.0 * math.sin(direction)
        target_point = [target_x, target_y, 30.0]
        if scouts:
            scout = scouts.pop(0)
            scout._assigned_target = None
            chase_point(scout, target_point, map_grid=map_grid)
        if interceptors:
            interceptor = interceptors.pop(0)
            interceptor._assigned_target = None
            chase_point(interceptor, target_point, map_grid=map_grid)


def update_chase_strategy(drones, offensive_targets, assignment_cfg=None, map_grid=None, base_positions=None):
    """输入一个无人机实例列表, 让防守方追击根据任务分配得到的目标进攻方, 进攻方追击传入的offensive_target_point

    参数说明：
    - drones: 无人机实例列表
    - offensive_target_point: 进攻方的公共目标点(即基地坐标)
    - assignment_cfg: 任务分配配置
    - map_grid: 可选的 MapGrid 实例。若提供则尝试使用 CBS 进行二维路径规划
    - base_position: 基地坐标 [x,y,z]，用于感知屏蔽计算（屏蔽扇区原点 + 防守方派遣起点）

    兼容性：当 map_grid 为 None 时，退化为原有的直接追逐逻辑。
    """
    # ========== 冷却机制初始化（只需一次） ==========
    if not hasattr(update_chase_strategy, "_last_cbs_time"):
        update_chase_strategy._last_cbs_time = 0.0
        update_chase_strategy._last_cbs_paths = None
    current_time = time.time()
    CBS_REPLAN_INTERVAL = 1.0   # 秒，可根据需要调整（0.5 ~ 2.0）
    # =============================================
    if base_positions is None:
        base_positions = []

    offensive = [d for d in drones if _is_offensive(d) and d.is_alive()]
    defensive = [d for d in drones if not _is_offensive(d) and d.is_alive()]

    # 收集活跃反辐射无人机
    jammers = _get_jammers(drones)
    # 收集活跃电子战干扰无人机
    ew_drones = _get_ew_drones(drones)

    # 应用 EW 干扰（标记防守方 _ew_jammed）
    _apply_ew_jamming(drones, ew_drones)
    # 检查 AntiEW 是否靠近 EW无人机并关闭干扰
    anti_ew_drones_list = [d for d in drones if isinstance(d, AntiEWDrone) and d.is_alive()]
    _check_anti_ew_disruption(ew_drones, anti_ew_drones_list)

    # 清理之前的分配标记
    for dd in defensive:
        try:
            dd._assigned_target = None
        except Exception:
            pass
    #先将无人机列表中的所有实例遍历, 将它们分为offensive和defensive两个列表
    # 进攻方：优先尝试轻量避障——若与目标在 XY 投影上不可直达，则用单体 A* 规划并追踪第一航点
    def _needs_obstacle_aware_path(pos, target, grid) -> bool:
        settings = _OFFENSIVE_AVOID_SETTINGS or {}
        if not settings.get("enabled", True):
            return False
        low_alt = float(settings.get("low_altitude_threshold", 30.0))
        if min(float(pos[2]), float(target[2])) > low_alt:
            return False
        if grid is None or not hasattr(grid, "world_to_grid") or not hasattr(grid, "obstacle_grid"):
            return False
        sample_count = max(2, int(settings.get("sample_count", 8)))
        corridor = max(0, int(settings.get("corridor_cells", 1)))
        dx = float(target[0]) - float(pos[0])
        dy = float(target[1]) - float(pos[1])
        for i in range(sample_count + 1):
            t = i / sample_count
            x = float(pos[0]) + dx * t
            y = float(pos[1]) + dy * t
            gx, gy = grid.world_to_grid(x, y)
            for ox in range(-corridor, corridor + 1):
                for oy in range(-corridor, corridor + 1):
                    nx = gx + ox
                    ny = gy + oy
                    if 0 <= nx < grid.width and 0 <= ny < grid.height:
                        if grid.obstacle_grid[nx, ny]:
                            return True
        return False

    # ---------- TankDrone 护卫 EW 无人机 ----------
    escorted_tanks = []
    if ew_drones:
        # 尚未被护卫的 EW 列表
        ew_needing_escort = list(ew_drones)
        tank_drones_list = [d for d in offensive if isinstance(d, TankDrone)]
        for tank in tank_drones_list:
            if not ew_needing_escort:
                break
            nearest_ew = min(ew_needing_escort,
                             key=lambda ew: sum((p - q) ** 2 for p, q in zip(tank.position, ew.position)))
            dist_to_ew = sum((p - q) ** 2 for p, q in zip(tank.position, nearest_ew.position)) ** 0.5
            if dist_to_ew <= nearest_ew.jamming_radius * 1.5:
                # Tank 跟随 EW 而非冲基地
                chase_target(tank, nearest_ew, map_grid=map_grid)
                tank._escorting_ew = nearest_ew
                escorted_tanks.append(tank)
                ew_needing_escort.remove(nearest_ew)

    for drone in offensive:
        # 已在护卫 EW 的 Tank 跳过常规逻辑
        if drone in escorted_tanks:
            continue

        if hasattr(drone, '_avoid_path') and drone._avoid_path:
            idx = getattr(drone, '_avoid_idx', 0)
            if idx < len(drone._avoid_path):
                target_wp = drone._avoid_path[idx]
                # 检查是否到达当前航点
                dist_to_wp = distance(drone.position, target_wp)
                WAYPOINT_REACHED = 5.0  # 米
                if dist_to_wp < WAYPOINT_REACHED:
                    idx += 1
                    drone._avoid_idx = idx
                    if idx < len(drone._avoid_path):
                        # 切换到下一个航点
                        chase_point(drone, drone._avoid_path[idx], map_grid=None)
                    else:
                        # 路径结束，清除路径标记并转为直接追逐基地
                        del drone._avoid_path
                        del drone._avoid_idx
                        # 重新计算最近的基地并追逐
                        if isinstance(offensive_targets, list) and offensive_targets and isinstance(offensive_targets[0], (list, tuple)):
                            try:
                                def _dist_sq(a, b):
                                    return sum((p - q) ** 2 for p, q in zip(a, b))
                                tgt = min(offensive_targets, key=lambda bp: _dist_sq(tuple(drone.position[:3]), bp))
                                chase_point(drone, tgt, map_grid=map_grid)
                            except Exception:
                                chase_point(drone, offensive_targets[0], map_grid=map_grid)
                        else:
                            chase_point(drone, offensive_targets, map_grid=map_grid)
                else:
                    # 尚未到达，继续追逐当前航点
                    chase_point(drone, target_wp, map_grid=None)
            else:
                # 无效路径，清除
                del drone._avoid_path
                del drone._avoid_idx

        used_light_plan = False
        if map_grid is not None:
            try:
                from algorithms.cbs_pathplan import line_of_sight_world, a_star_plan_world
            except Exception:
                line_of_sight_world = None
                a_star_plan_world = None

            try:
                pos = tuple(drone.position[:3])
                needs_plan = False
                # 确定该进攻机的目标基地（最近的活跃基地）
                if isinstance(offensive_targets, list) and offensive_targets and isinstance(offensive_targets[0], (list, tuple)):
                    try:
                        # 选择最近的基地
                        def _dist_sq(a, b):
                            return sum((p - q) ** 2 for p, q in zip(a, b))
                        target_base = min(offensive_targets, key=lambda bp: _dist_sq(pos, bp))
                    except Exception:
                        target_base = offensive_targets[0]
                else:
                    target_base = offensive_targets

                if line_of_sight_world is not None and not line_of_sight_world(pos, target_base, map_grid):
                    needs_plan = True
                if not needs_plan and _needs_obstacle_aware_path(pos, target_base, map_grid):
                    needs_plan = True
                if needs_plan:
                    if a_star_plan_world is not None:
                        path = a_star_plan_world(pos, target_base, map_grid, max_time_ms=50)
                        if path:
                            try:
                                drone._avoid_path = path
                                drone._avoid_idx = 1 if len(path) > 1 else 0
                                next_wp = path[drone._avoid_idx]
                                chase_point(drone, next_wp, map_grid=None)
                                used_light_plan = True
                            except Exception:
                                used_light_plan = False
            except Exception:
                used_light_plan = False

        if not used_light_plan:
            # 运行时若 offensive_targets 是基地列表，则追击最近的基地
            if isinstance(offensive_targets, list) and offensive_targets and isinstance(offensive_targets[0], (list, tuple)):
                try:
                    def _dist_sq(a, b):
                        return sum((p - q) ** 2 for p, q in zip(a, b))
                    tgt = min(offensive_targets, key=lambda bp: _dist_sq(tuple(drone.position[:3]), bp))
                    chase_point(drone, tgt, map_grid=map_grid)
                except Exception:
                    chase_point(drone, offensive_targets[0], map_grid=map_grid)
            else:
                chase_point(drone, offensive_targets, map_grid=map_grid)

    # -------- 防守方行为 --------
    if not defensive:
        return

    # EW 干扰：分离未受干扰和受干扰的防守方
    jammed_defenders = [d for d in defensive if getattr(d, '_ew_jammed', False)]
    free_defenders = [d for d in defensive if not getattr(d, '_ew_jammed', False)]
    # AntiEW 无人机不参与常规任务分配（它们已经指定追击 EW）
    free_defenders = [d for d in free_defenders if not isinstance(d, AntiEWDrone)]

    # 被 EW 干扰的拦截机：强迫追击最近 EW 无人机（重点摧毁）
    for jd in list(jammed_defenders):
        if isinstance(jd, InterceptorDrone) and ew_drones:
            nearest_ew = min(ew_drones,
                             key=lambda ew: sum((p - q) ** 2 for p, q in zip(jd.position, ew.position)))
            jd._assigned_target = nearest_ew
            chase_target(jd, nearest_ew, map_grid=map_grid)
            jammed_defenders.remove(jd)
        else:
            # 其他被干扰防守方暂时减速（停在原地）
            jd.set_velocity([0.0, 0.0, 0.0])

    # AntiEW 无人机：追击最近的 EW 无人机（免疫干扰）
    for aew in anti_ew_drones_list:
        if aew.is_alive() and ew_drones:
            nearest_ew = min(ew_drones,
                             key=lambda ew: sum((p - q) ** 2 for p, q in zip(aew.position, ew.position)))
            chase_target(aew, nearest_ew, map_grid=map_grid)

    # 筛选防守方"可见"的进攻方（屏蔽扇区内不可见，反辐射自身不参与分配）
    visible_offensive = [
        d for d in offensive
        if not isinstance(d, AntiRadiationDrone) and _is_visible_to_defense(d, base_positions, jammers)
    ]

    # 拦截机优先分配 EW 无人机（干扰加权）
    if visible_offensive and free_defenders:
        if assignment_cfg is None:
            try:
                from algorithms.auction_assign import ImprovedAuctionConfig
                assignment_cfg = ImprovedAuctionConfig()
            except Exception:
                assignment_cfg = None
        # 将基地位置提供给拍卖算法以用于距离归一化
        try:
            import algorithms.auction_assign as _aa
            _aa.set_base_positions(base_positions if base_positions else None)
        except Exception:
            pass
        # 构建 jammed_mask 传入拍卖（双重保险：即使 jammed defender 误入 free_defenders）
        import numpy as np
        _jammed = np.zeros(len(free_defenders), dtype=bool)
        for _i, _d in enumerate(free_defenders):
            if getattr(_d, '_ew_jammed', False):
                _jammed[_i] = True
        try:
            from algorithms.auction_assign import greedy_assignment
            assignment = greedy_assignment(
                free_defenders,
                visible_offensive,
                config=assignment_cfg,
                map_grid=map_grid,
                jammed_mask=_jammed if _jammed.any() else None,
            )
        except Exception:
            assignment = {d.name: None for d in free_defenders}
        name_to_defender = {d.name: d for d in free_defenders}
        pairs = []
        defenders_order = []
        for def_name, target in assignment.items():
            defender = name_to_defender.get(def_name)
            if defender is None:
                continue
            if target is not None:
                try:
                    defender._assigned_target = target
                except Exception:
                    pass
                goal_pos = target.position if hasattr(target, "position") else None
                if goal_pos is not None:
                    pairs.append((defender, goal_pos))
                    defenders_order.append(defender)
                else:
                    chase_target(defender, target)
            else:
                try:
                    defender._assigned_target = None
                except Exception:
                    pass
                defender.set_velocity([0.0, 0.0, 0.0])

        if map_grid is not None and pairs:
            # 检查是否需要重新规划（冷却控制）
            need_replan = (current_time - update_chase_strategy._last_cbs_time) >= CBS_REPLAN_INTERVAL
            
            if need_replan:
                try:
                    from algorithms.cbs_pathplan import cbs_plan_paths
                    plan_inputs = [(p[0], p[1]) for p in pairs]
                    world_paths = cbs_plan_paths(plan_inputs, map_grid=map_grid)
                    # 更新缓存
                    update_chase_strategy._last_cbs_time = current_time
                    update_chase_strategy._last_cbs_paths = world_paths
                except Exception:
                    world_paths = []
                    update_chase_strategy._last_cbs_paths = []
            else:
                # 使用上次缓存的路径，但需检查长度是否匹配
                if (update_chase_strategy._last_cbs_paths is not None and 
                    len(update_chase_strategy._last_cbs_paths) == len(defenders_order)):
                    world_paths = update_chase_strategy._last_cbs_paths
                else:
                    # 长度不匹配，强制重规划（将 need_replan 设为 True）
                    need_replan = True
                    # 重新执行规划（可复制 need_replan 分支的代码，或直接递归调用，简单起见可以再次调用 cbs_plan_paths）
                    try:
                        from algorithms.cbs_pathplan import cbs_plan_paths
                        plan_inputs = [(p[0], p[1]) for p in pairs]
                        world_paths = cbs_plan_paths(plan_inputs, map_grid=map_grid)
                        update_chase_strategy._last_cbs_time = current_time
                        update_chase_strategy._last_cbs_paths = world_paths
                    except Exception:
                        world_paths = []
                        update_chase_strategy._last_cbs_paths = []

            if world_paths:
                for defender, path in zip(defenders_order, world_paths):
                    if path and len(path) >= 2:
                        next_wp = path[1]
                        chase_point(defender, next_wp)
                        try:
                            defender._cbs_path = path
                            defender._cbs_goal = path[-1]
                        except Exception:
                            pass
                    elif path and len(path) == 1:
                        chase_point(defender, path[0])
                    else:
                        target_obj = assignment.get(defender.name)
                        if target_obj is not None:
                            chase_target(defender, target_obj)
            else:
                # 规划失败或无路径，退化为直接追逐
                for def_name, target in assignment.items():
                    defender = name_to_defender.get(def_name)
                    if defender is None:
                        continue
                    if target is not None:
                        chase_target(defender, target)
                    else:
                        defender.set_velocity([0.0, 0.0, 0.0])
    elif free_defenders:
        for defender in free_defenders:
            defender.set_velocity([0.0, 0.0, 0.0])

    # 向屏蔽方向派遣未分配的防守方
    _dispatch_defenders_to_jammed_directions(
        defensive=defensive,
        jammers=jammers,
        base_positions=base_positions,
        map_grid=map_grid,
    )


#=================暂行无人机生成策略，之后要被任务分配算法替代============================================

def spawn_random_drone(config: dict, drones: list, base_positions: list | None = None) -> None:
    """根据当前进攻方数量随机生成进攻方无人机。
    - AttackDrone/TankDrone 在存在屏蔽扇区时偏向扇区方向生成
    - 反辐射无人机以较低概率（约 15%）生成
    """
    if base_positions is None:
        base_positions = []

    offensive = [d for d in drones if _is_offensive(d) and d.is_alive()]
    sim_cfg = config.get("simulation", {})
    max_offensive = int(sim_cfg.get("max_offensive", 6))
    if len(offensive) >= max_offensive:
        return

    jammers = _get_jammers(drones)
    ard_prob = float(config.get("simulation", {}).get("anti_radiation_spawn_prob", 0.15))
    ew_prob = float(config.get("simulation", {}).get("ew_spawn_prob", 0.10))
    roll = random.random()

    # 反辐射无人机
    if roll < ard_prob:
        z = random.uniform(100.0, 250.0)
        direction = random.choice(['top', 'bottom', 'left', 'right'])
        if direction == 'top':
            x = random.uniform(-500, 500)
            y = random.uniform(520, 550)
        elif direction == 'bottom':
            x = random.uniform(-500, 500)
            y = random.uniform(-550, -520)
        elif direction == 'left':
            x = random.uniform(-550, -520)
            y = random.uniform(-500, 500)
        else:
            x = random.uniform(520, 550)
            y = random.uniform(-500, 500)
        position = [x, y, z]
        name = f"AntiRad-{random.randint(100, 999)}"
        create_anti_radiation_drone(name, position, config, drones)
        LOGGER.info("Spawned AntiRadiation drone: %s at %s", name, position)
    elif roll < ard_prob + ew_prob:
        # 电子战干扰无人机：从地图边界生成
        z = random.uniform(100.0, 250.0)
        direction = random.choice(['top', 'bottom', 'left', 'right'])
        if direction == 'top':
            x = random.uniform(-500, 500)
            y = random.uniform(520, 550)
        elif direction == 'bottom':
            x = random.uniform(-500, 500)
            y = random.uniform(-550, -520)
        elif direction == 'left':
            x = random.uniform(-550, -520)
            y = random.uniform(-500, 500)
        else:
            x = random.uniform(520, 550)
            y = random.uniform(-500, 500)
        position = [x, y, z]
        name = f"EW-{random.randint(100, 999)}"
        create_ew_drone(name, position, config, drones)
        LOGGER.info("Spawned EW drone: %s at %s", name, position)
    else:
        drone_type = random.choice(["attack", "tank"])
        if jammers and random.random() < 0.65:
            jammer = random.choice(jammers)
            if jammer.jamming_direction is not None:
                direction_angle = jammer.jamming_direction
                half = jammer.jamming_sector_half_angle
                angle = direction_angle + random.uniform(-half, half)
                dist = random.uniform(250.0, 450.0)
                z = random.uniform(100.0, 250.0)
                # 选择该 jammer 绑定的基地作为参考点（退化使用第一个基地）
                base_ref = getattr(jammer, "_base_position", (base_positions[0] if base_positions else [0.0, 0.0, 0.0]))
                x = base_ref[0] + dist * math.cos(angle)
                y = base_ref[1] + dist * math.sin(angle)
                position = [x, y, z]
                name = f"{drone_type.capitalize()}-{random.randint(100, 999)}"
                if drone_type == "attack":
                    create_attack_drone(name, position, config, drones)
                elif drone_type == "tank":
                    create_tank_drone(name, position, config, drones)
                return

        z = random.uniform(100.0, 250.0)
        direction = random.choice(['top', 'bottom', 'left', 'right'])
        if direction == 'top':
            x = random.uniform(-500, 500)
            y = random.uniform(520, 550)
        elif direction == 'bottom':
            x = random.uniform(-500, 500)
            y = random.uniform(-550, -520)
        elif direction == 'left':
            x = random.uniform(-550, -520)
            y = random.uniform(-500, 500)
        else:
            x = random.uniform(520, 550)
            y = random.uniform(-500, 500)
        position = [x, y, z]
        name = f"{drone_type.capitalize()}-{random.randint(100, 999)}"
        if drone_type == "attack":
            create_attack_drone(name, position, config, drones)
        elif drone_type == "tank":
            create_tank_drone(name, position, config, drones)

def balance_defenders(config: dict, drones: list) -> None:
    """基于未匹配攻击方数量补充防守方。

    只统计可用防守方（未被 EW 干扰、非 AntiEW），与可见进攻方数量对比。
    """
    offensive = [d for d in drones if _is_offensive(d) and d.is_alive()]
    # 可用防守方：存活、未被干扰(或被干扰但属于拦截机)、非 AntiEW（AntiEW 不参与常规分配）
    from drones.defensive.anti_ew_drone import AntiEWDrone
    from drones.defensive.interceptor_drone import InterceptorDrone

    available_defenders = [
        d for d in drones
        if not _is_offensive(d)
        and d.is_alive()
        and not isinstance(d, AntiEWDrone)
        and (not getattr(d, '_ew_jammed', False) or isinstance(d, InterceptorDrone))
    ]
    # 需要匹配的攻击方数量 = 可见进攻方（排除 ARD，它在自我毁灭路径上）
    target_count = len([a for a in offensive if not isinstance(a, AntiRadiationDrone)])
    need = max(0, target_count - len(available_defenders))
    if need <= 0:
        return
    for _ in range(need):
        drone_type = random.choice(["scout", "interceptor"])
        position = [0.0, 0.0, 0.0]
        name = f"{drone_type.capitalize()}-{random.randint(100,999)}"
        if drone_type == "scout":
            create_scout_drone(name, position, config, drones)
        else:
            create_interceptor_drone(name, position, config, drones)


def _build_assignment_config(config: dict):
    """从主配置中构建任务分配配置，支持在 config.yaml 中直接调参。"""
    from algorithms.auction_assign import ImprovedAuctionConfig

    raw = config.get("assignment", {})
    if not isinstance(raw, dict):
        LOGGER.warning("Config key 'assignment' must be a mapping. Using defaults.")
        return ImprovedAuctionConfig()

    valid_keys = {item.name for item in fields(ImprovedAuctionConfig)}
    unknown_keys = sorted(set(raw.keys()) - valid_keys)
    if unknown_keys:
        LOGGER.warning("Unknown assignment config keys ignored: %s", ", ".join(unknown_keys))

    kwargs = {key: raw[key] for key in valid_keys if key in raw}
    try:
        return ImprovedAuctionConfig(**kwargs)
    except (TypeError, ValueError) as exc:
        LOGGER.warning("Invalid assignment config detected, using defaults: %s", exc)
        return ImprovedAuctionConfig()

#==================================================================================================


def run_simulation(config: dict):
    """运行仿真主循环"""
    global _ASSIGNMENT_CONFIG
    _ASSIGNMENT_CONFIG = _build_assignment_config(config)
    sim_cfg = config.get("simulation", {})
    global _OFFENSIVE_AVOID_SETTINGS
    _OFFENSIVE_AVOID_SETTINGS = {
        "enabled": bool(sim_cfg.get("offensive_obstacle_avoid", True)),
        "low_altitude_threshold": float(sim_cfg.get("offensive_avoid_low_alt_threshold", 30.0)),
        "corridor_cells": int(sim_cfg.get("offensive_avoid_corridor_cells", 1)),
        "sample_count": int(sim_cfg.get("offensive_avoid_sample_count", 8)),
    }

    drones = []
    #创建无人机初始团队
    base_manager = BaseManager(config)
    #创建基地管理器实例, 从配置中读取基地的相关参数
    # 雷达传感器已移除，仿真不再执行探测扫描
    display = None
    if Open3DDisplay is not None:
        try:
            obj_model_path = str(Path(__file__).resolve().parent / "Data" / "maps" / "3d66.com_JJI54558918189.obj")
            if os.path.exists(obj_model_path):
                display = Open3DDisplay(map_size=(config["map"]["width"], config["map"]["height"]),
                                        obj_model_path=obj_model_path, enable_trails=True, enable_dashed_lines=True)
            else:
                display = Open3DDisplay(map_size=(config["map"]["width"], config["map"]["height"]), enable_trails=True, enable_dashed_lines=True)
            display.open_window()
            display._init_map_data()
        except Exception as exc:
            LOGGER.warning("Open3D initialization failed, running headless: %s", exc)
            display = None

    fps = config["simulation"]["fps"]
    frame_time = 1.0 / fps
    start_time = time.time()
    duration = config["simulation"]["duration"]
    # 生成间隔支持从配置读取，便于做小规模仿真
    spawn_min = config.get("simulation", {}).get("spawn_interval_min", 1.0)
    spawn_max = config.get("simulation", {}).get("spawn_interval_max", 3.0)
    next_spawn_time = random.uniform(spawn_min, spawn_max)
    spawn_timer = 0.0
    # 反辐射无人机自毁检查间隔（每 0.5 秒集中检查一次）
    ard_check_interval = 0.5
    ard_check_timer = 0.0
    # 读取基地位置（可能为多个），优先使用 BaseManager 提供的列表
    try:
        base_positions = base_manager.get_positions()
    except Exception:
        base_positions = [config.get("base", {}).get("position", [0.0, 0.0, 0.0])]

    # 将基地位置传递给拍卖算法以便距离归一化
    try:
        import algorithms.auction_assign as _aa
        _aa.set_base_positions(base_positions)
    except Exception:
        pass

    # ---------- 终端按键重置路径规划 ----------
    # 仅在 Windows 下尝试导入 msvcrt，非 Windows 则禁用按键功能
    try:
        import msvcrt
        _keyboard_available = True
    except ImportError:
        _keyboard_available = False
        LOGGER.warning("msvcrt not available (non-Windows), terminal reset key disabled")

    def reset_all_paths():
        """强制重置所有无人机的路径规划状态"""
        LOGGER.info("Manual reset: clearing all path planning states")
        for drone in drones:
            # 进攻方 A* 路径
            if hasattr(drone, '_avoid_path'):
                delattr(drone, '_avoid_path')
            if hasattr(drone, '_avoid_idx'):
                delattr(drone, '_avoid_idx')
            # PathTracker 缓存
            if hasattr(drone, '_path_tracker'):
                drone._path_tracker = None
            # 防守方 CBS 路径
            if hasattr(drone, '_cbs_path'):
                delattr(drone, '_cbs_path')
            if hasattr(drone, '_cbs_goal'):
                delattr(drone, '_cbs_goal')
            # 分配目标标记
            if hasattr(drone, '_assigned_target'):
                drone._assigned_target = None
        # 重置全局 CBS 缓存
        if hasattr(update_chase_strategy, '_last_cbs_time'):
            update_chase_strategy._last_cbs_time = 0.0
        if hasattr(update_chase_strategy, '_last_cbs_paths'):
            update_chase_strategy._last_cbs_paths = None
        LOGGER.info("Reset completed")
    # ------------------------------------------

    frame_counter = 0
    total_frame_time = 0.0
    try:
        while (duration <= 0 or time.time() - start_time < duration) and not base_manager.is_destroyed():
            # 当 duration<=0 时表示无限运行，否则按配置时长运行, 同时如果基地被摧毁了也要结束仿真
            frame_start = time.perf_counter()   # 修正1：定义帧开始时间
            #1.碰撞检测与处理
            t1 = time.perf_counter()
            collisions = detect_collisions(drones)  # 收集这一帧中发生碰撞的无人机的列表"""
            if collisions:
                resolve_collisions(collisions)  #如果在这一帧中发生了碰撞, 那么对发生碰撞的无人机对执行resolve_collisions中的碰撞处理函数"""
            t_collision = time.perf_counter() - t1

            #2. 更新每架无人机的电量, 如果电量耗尽则触发坠毁流程
            t1 = time.perf_counter()
            for drone in drones:
                drone.update_battery(frame_time)
            t_battery = time.perf_counter() - t1

            #3. 更新坠毁无人机的状态
            for drone in drones:
                if drone.destroyed:  #如果无人机被判定摧毁
                    if drone.falling:   #如果无人机还在"falling"的过程中
                        drone.update_fall(frame_time)   #更新它的falling状态
                    elif drone.impact:  #如果无人机已经落地
                        drone.update_death_timer(frame_time)    #更新death_timer(播放爆炸动画)

            #4.剔除爆炸结束的无人机，将其从整个仿真中清除
            drones = [d for d in drones if not d.should_remove()]
            #剔除drones列表中所有被 .should_move函数判断为"应该被移除"的无人机, 即更新drone列表"""

            #5.获取存活无人机列表
            alive_drones = [d for d in drones if not d.destroyed]

            #5.5 反辐射无人机自毁检查（每 0.5 秒集中检查一次）
            ard_check_timer += frame_time
            if ard_check_timer >= ard_check_interval:
                ard_check_timer = 0.0
                for drone in alive_drones:
                    if isinstance(drone, AntiRadiationDrone) and drone.is_alive():
                        # 为每架 ARD 选择最近的基地作为自毁判定原点
                        if base_positions:
                            try:
                                def _dist_sq(a, b):
                                    return sum((p - q) ** 2 for p, q in zip(a, b))
                                base_ref = min(base_positions, key=lambda bp: _dist_sq(drone.position, bp))
                            except Exception:
                                base_ref = base_positions[0]
                        else:
                            base_ref = config.get("base", {}).get("position", [0.0, 0.0, 0.0])
                        if drone.check_self_destruct(base_ref):
                            LOGGER.info(
                                "AntiRadiation drone %s self-destructed near base at %s",
                                drone.name, drone.position,
                            )

            #6. 更新存活无人机的追逐策略
            # 将 display.map_grid 传入路径规划模块；若 display 不存在则传入 None
            t1 = time.perf_counter()
            update_chase_strategy(
                alive_drones,
                base_positions,
                _ASSIGNMENT_CONFIG,
                map_grid=(display.map_grid if display is not None else None),
                base_positions=base_positions,
            ) # 进攻方的目标点为基地列表（每架机将选择最近基地）
            t_chase = time.perf_counter() - t1

            #6.5 检测 EW 无人机，生成 AntiEW 应对
            _spawn_anti_ew_if_needed(config, alive_drones, base_positions)
            # 更新每架无人机的追逐策略 """

            #7. 更新存活无人机的位置
            t1 = time.perf_counter()
            for drone in alive_drones:
                move_drone(drone, frame_time, map_grid=(display.map_grid if display else None))
            #对在alive_drone列表中(也就是确认存活的)无人机, 执行位置更新操作"""
            t_move = time.perf_counter() - t1

            #8. 基地碰撞检测
            t1 = time.perf_counter()
            if base_manager.check_collisions(drones):
                LOGGER.info("Base destroyed! Attackers win!")
                break
            t_base_collision = time.perf_counter() - t1

            #9. 随机生成进攻无人机
            spawn_timer += frame_time  #随机生成进攻方无人机
            if spawn_timer >= next_spawn_time:
                spawn_timer = 0.0
                next_spawn_time = random.uniform(spawn_min, spawn_max)
                if random.random() < 0.9:
                    spawn_random_drone(config, drones, base_positions)

            #10. 平衡防守方数量
            balance_defenders(config, drones) #每帧立即平衡防守方数量, 直到防守方数量 >= 进攻方数量"""

            detections = []
            t1 = time.perf_counter()
            #雷达模块已移除，进攻方目标全局可知
            LOGGER.debug("Detected %d objects", len(detections))
            if display is not None:
                # 传入总血量百分比与基地位置供可视化显示（HUD 使用 0-100%）
                total_pct = 100.0 * base_manager.get_health() / max(base_manager.get_max_health(), 1e-6)
                display.update(drones, detections, total_pct, base_positions)
                if not display.is_open:
                    LOGGER.info("Visualization window closed, ending simulation.")
                    break
                # Run GUI event loop tick
                if not display.app.run_one_tick():
                    LOGGER.info("GUI event loop ended.")
                    break
            t_vis = time.perf_counter() - t1
    
            frame_end = time.perf_counter()
            frame_total = frame_end - frame_start

            # 慢帧警告（包含所有计时指标）
            if frame_total > 0.05:   # 50ms
                LOGGER.warning(
                    f"Slow frame: {frame_total*1000:.1f}ms | "
                    f"Collision:{t_collision*1000:.2f}ms Battery:{t_battery*1000:.2f}ms "
                    f"Chase:{t_chase*1000:.2f}ms Move:{t_move*1000:.2f}ms "
                    f"BaseCollision:{t_base_collision*1000:.2f}ms Vis:{t_vis*1000:.2f}ms"
                )
            # ---------- 终端按键检测（重置路径）----------
            if _keyboard_available and msvcrt.kbhit():
                try:
                    key = msvcrt.getch().decode('ascii').lower()
                    if key == 'r':
                        reset_all_paths()
                except Exception:
                    pass
            # -------------------------------------------

            time.sleep(frame_time)
    finally:
        if display is not None:
            display.close_window()
            time.sleep(0.2)   # 增加这一行，等待 Open3D 清理
            #如果上面的try中发生错误, 则直接执行finally中的代码, 使仿真出错时可以正常退出程序

if __name__ == "__main__":
    #此处为程序的入口
    #如果此文件被直接运行, 则可以成功进入程序入口
    #创建一个表示文件路径的对象, 指向当前目录下的config.yaml文件
    #打开config.yaml文件, 返回一个python字典, 包含地图尺寸, 雷达参数, 无人机熟悉, 仿真时长, 仿真帧率等信息
    #在之前创建过的LOGGER中输出一条info等级的日志: 仿真开始
    #运行仿真
    #输出一条info等级的日志: 仿真结束
  
    config_path = Path(__file__).resolve().parent / "config.yaml"
    config = load_config(config_path)
    LOGGER.info("Starting AirCombat simulation")
    run_simulation(config)
    LOGGER.info("Simulation ended")
