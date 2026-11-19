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
from Controller.single_control import chase_target, chase_point, move_drone, PathFollower
from utils.logger import get_logger
from utils.geometry import distance
try:
    from utils.map_grid import MapGrid
except Exception:  # numpy 等依赖缺失时退化为无地图模式
    MapGrid = None
try:
    from utils.map_loader import generate_buildings
except Exception:  # 地图生成失败时仍可退化为无地图模式
    generate_buildings = None
from base.base_manager import BaseManager

LOGGER = get_logger(__name__)

# 缓存一些开销较大的算法模块, 避免在每帧中重复导入
try:
    import algorithms.auction_assign as _auction_assign
except Exception:
    _auction_assign = None
try:
    import algorithms.cbs_pathplan as _cbs_pathplan
except Exception:
    _cbs_pathplan = None

# Open3D 可视化：导入失败（未安装 open3d 等）时自动退化为无界面模式
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
    """ 寻找最近的敌人, 传入代表自己的实例(drone), 与可能要追逐的"候选"的敌方无人机的列表candidates"""
    best = None
    best_dist = float("inf")
    for other in candidates:
        if other is drone or not other.is_alive():
            continue
        dist = sum((p - q) ** 2 for p, q in zip(drone.position, other.position))
        if dist < best_dist:
            best_dist = dist
            best = other
    return best


def _build_map_grid(config: dict):
    """按配置生成规划网格；任何一步失败都返回 None，仿真退化为无地图模式。"""
    if MapGrid is None or generate_buildings is None:
        LOGGER.warning("MapGrid/map_loader unavailable, path planning disabled")
        return None
    try:
        map_cfg = config.get("map", {})
        width = float(map_cfg.get("width", 1000))
        height = float(map_cfg.get("height", 1000))
        cell_size = float(map_cfg.get("path_cell_size", 5.0))
        buildings = generate_buildings(config=config)
        grid = MapGrid(
            buildings,
            bounds=(-width / 2.0, -height / 2.0, width / 2.0, height / 2.0),
            cell_size=cell_size,
        )
        LOGGER.info("Path planning grid: %dx%d cell=%.1fm buildings=%d",
                    grid.width, grid.height, cell_size, len(buildings))
        return grid
    except Exception as exc:
        LOGGER.warning("Failed to build map grid, path planning disabled: %s", exc)
        return None


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

def _spawn_anti_ew_if_needed(config: dict, drones: list, base_positions: list | None,
                             map_grid=None):
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

    def _anti_ew_spawn_pos(base):
        if map_grid is None:
            return [float(base[0]), float(base[1]), 50.0]
        try:
            for _ in range(24):
                angle = random.uniform(0.0, 2.0 * math.pi)
                radius = random.uniform(15.0, 60.0)
                x = float(base[0]) + radius * math.cos(angle)
                y = float(base[1]) + radius * math.sin(angle)
                if not map_grid.is_occupied(x, y):
                    return [x, y, 50.0]
            for _ in range(160):
                gx = random.randrange(map_grid.width)
                gy = random.randrange(map_grid.height)
                if not map_grid.is_grid_occupied(gx, gy):
                    x, y = map_grid.grid_to_world(gx, gy)
                    return [float(x), float(y), 50.0]
        except Exception:
            pass
        return [float(base[0]), float(base[1]), 50.0]

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
            spawn_base = config.get("base", {}).get("position", [0.0, 0.0, 0.0])
        spawn_pos = _anti_ew_spawn_pos(spawn_base)
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
                    aew.name, ew.name, dist,
                )


# ====== 以下为原有 ARD 感知屏蔽工具 ======

def _get_jammers(drones: list):
    """返回所有存活且屏蔽已初始化的反辐射无人机。"""
    return [
        d for d in drones
        if isinstance(d, AntiRadiationDrone) and d.jamming_active
    ]


def _is_visible_to_defense(drone, base_positions: list | None, jammers: list,
                           defenders: list | None = None, proximity: float = 120.0) -> bool:
    """进攻方无人机是否对防守方可见。

    - 近距离（默认 120 m）视为光学/红外可见，不受 ARD 屏蔽影响；
    - 否则在任一屏蔽扇区内即不可见。
    对于多基地场景，ARD 在创建时已固化其绑定的基地，is_in_jammed_sector
    会使用该绑定基地作为原点；因此此处无需传入统一的 base_position。
    """
    if defenders:
        for d in defenders:
            if getattr(d, "is_alive", lambda: False)() and distance(drone.position, d.position) <= proximity:
                return True
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

    def _send(drone, target_point):
        # 直接朝派遣点飞：PathTracker 会在基地上空反复爬升；到点后停下，避免来回抽动
        if distance(drone.position, target_point) < 30.0:
            drone.set_velocity([0.0, 0.0, 0.0])
        else:
            chase_point(drone, target_point, map_grid=None)

    # 为每个“基地 + 屏蔽方向”只派遣一次，避免同一方向堆多架
    seen_dirs = set()
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
        key = (round(float(direction), 2), round(float(base_pos[0]), 1), round(float(base_pos[1]), 1))
        if key in seen_dirs:
            continue
        seen_dirs.add(key)
        target_x = base_pos[0] + 300.0 * math.cos(direction)
        target_y = base_pos[1] + 300.0 * math.sin(direction)
        target_point = [target_x, target_y, 30.0]
        if scouts:
            scout = scouts.pop(0)
            scout._assigned_target = None
            scout._path_follower = None
            _send(scout, target_point)
        if interceptors:
            interceptor = interceptors.pop(0)
            interceptor._assigned_target = None
            interceptor._path_follower = None
            _send(interceptor, target_point)


def update_path_followers(drones, sim_time):
    """每帧推进 CBS 路径；路径走完后回退到当前分配目标/悬停。

    与任务分配/重规划解耦，使策略可以降频执行，而防守方仍按时间表平滑移动。
    """
    for drone in drones:
        # 碰撞后的短暂击退窗口：保留反向速度先分开，不要被 CBS 路径立刻覆盖
        if getattr(drone, "_collision_knockback_until", 0.0) > time.time():
            continue
        follower = getattr(drone, "_path_follower", None)
        if follower is None:
            continue
        if getattr(drone, "_assigned_target", None) is None:
            # 目标被撤销/正在被派往屏蔽方向：不要再用旧路径覆盖新速度
            drone._path_follower = None
            continue
        if follower.update(sim_time):
            continue
        target = getattr(drone, "_assigned_target", None)
        if target is not None and target.is_alive():
            chase_target(drone, target)
        else:
            drone.set_velocity([0.0, 0.0, 0.0])


def update_chase_strategy(drones, offensive_targets, assignment_cfg=None, map_grid=None,
                          base_positions=None, sim_time=None, plan_speed=30.0):
    """输入一个无人机实例列表, 让防守方追击根据任务分配得到的目标进攻方, 进攻方追击传入的offensive_target_point

    参数说明：
    - drones: 无人机实例列表
    - offensive_target_point: 进攻方的公共目标点(即基地坐标)
    - assignment_cfg: 任务分配配置
    - map_grid: 可选的 MapGrid 实例。若提供则尝试使用 CBS 进行二维路径规划
    - base_position: 基地坐标 [x,y,z]，用于感知屏蔽计算（屏蔽扇区原点 + 防守方派遣起点）
    - sim_time: 仿真时间（秒）。提供时按仿真时间做 CBS 重规划冷却，否则退回墙上时钟
    - plan_speed: CBS 路径的执行速度（米/秒），全体防守方统一，保证时间表可比

    兼容性：当 map_grid 为 None 时，退化为原有的直接追逐逻辑。
    """
    # ========== 冷却机制初始化（只需一次） ==========
    if not hasattr(update_chase_strategy, "_last_cbs_time"):
        update_chase_strategy._last_cbs_time = -1e9
        update_chase_strategy._last_cbs_paths = None
    if not hasattr(update_chase_strategy, "_last_cbs_signature"):
        update_chase_strategy._last_cbs_signature = None
    current_time = sim_time if sim_time is not None else time.time()
    # 仿真重新开始（时间回退）时清空上一次的 CBS 缓存
    if current_time < update_chase_strategy._last_cbs_time:
        update_chase_strategy._last_cbs_time = -1e9
        update_chase_strategy._last_cbs_paths = None
        update_chase_strategy._last_cbs_signature = None
    CBS_REPLAN_INTERVAL = 1.0   # 仿真秒，可根据需要调整（0.5 ~ 2.0）
    LOCK_TTL = 8.0              # 目标锁定最长时间（仿真秒），避免无休止追一个目标
    BASE_GUARD_RADIUS = 300.0   # 进攻方进入基地该半径内，触发终端防御（强制最近防守方拦截）
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

    # 清理之前的分配标记；同时记住上一帧的目标，短暂丢失可见性时继续追击
    for dd in defensive:
        prev = getattr(dd, "_assigned_target", None)
        if prev is not None and getattr(prev, "is_alive", lambda: False)():
            if getattr(dd, "_locked_target", None) is not prev:
                dd._lock_start = current_time
            dd._locked_target = prev
            dd._locked_time = current_time
        elif prev is not None:
            dd._locked_target = None
            dd._locked_time = None
        try:
            dd._assigned_target = None
        except Exception:
            pass

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
            # 使用模块级缓存, 避免每帧重复导入
            try:
                if _cbs_pathplan is not None:
                    line_of_sight_world = _cbs_pathplan.line_of_sight_world
                    a_star_plan_world = _cbs_pathplan.a_star_plan_world
                else:
                    line_of_sight_world = None
                    a_star_plan_world = None
            except Exception:
                line_of_sight_world = None
                a_star_plan_world = None

            try:
                pos = tuple(drone.position[:3])
                needs_plan = False
                # 确定该进攻机的目标基地（最近的活跃基地）
                used_light_plan = False
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
    claimed_ew_ids = set()
    claimed_local_ids = set()

    def _local_base_threat(defender):
        """被干扰、无法参与拍卖的防守方：只在基地附近做本地自主拦截。"""
        if not base_positions:
            return None
        nearest_base = min(base_positions,
                           key=lambda bp: distance(defender.position, bp))
        if distance(defender.position, nearest_base) > 350.0:
            return None
        threats = [
            a for a in offensive
            if not isinstance(a, AntiRadiationDrone)
            and a.is_alive()
            and id(a) not in claimed_local_ids
            and distance(a.position, nearest_base) <= BASE_GUARD_RADIUS
        ]
        if not threats:
            return None
        # 本地防御优先拦“最接近基地”的威胁，而不是离自己最近的
        return min(threats, key=lambda a: distance(a.position, nearest_base))

    def _near_any_base(obj) -> bool:
        if not base_positions:
            return False
        return min(distance(obj.position, bp) for bp in base_positions) <= BASE_GUARD_RADIUS

    for jd in list(jammed_defenders):
        candidates = [ew for ew in ew_drones if id(ew) not in claimed_ew_ids]
        if isinstance(jd, InterceptorDrone) and candidates:
            nearest_ew = min(candidates,
                             key=lambda ew: sum((p - q) ** 2 for p, q in zip(jd.position, ew.position)))
            claimed_ew_ids.add(id(nearest_ew))
            jd._assigned_target = nearest_ew
            chase_target(jd, nearest_ew, map_grid=map_grid)
            jammed_defenders.remove(jd)
        else:
            # EW 切断了协同链路，但基地附近的本地自卫仍然可以做
            prior = getattr(jd, "_locked_target", None)
            jd_near_base = bool(base_positions) and min(
                distance(jd.position, bp) for bp in base_positions) <= 350.0
            if (prior is not None and getattr(prior, "is_alive", lambda: False)()
                    and id(prior) not in claimed_local_ids and _near_any_base(prior)
                    and jd_near_base):
                local_threat = prior
            else:
                local_threat = _local_base_threat(jd)
            if local_threat is not None:
                claimed_local_ids.add(id(local_threat))
                jd._assigned_target = local_threat
                jd._locked_target = local_threat
                jd._lock_start = current_time
                jd._path_follower = None
                chase_target(jd, local_threat, map_grid=None)
            else:
                jd.set_velocity([0.0, 0.0, 0.0])

    # AntiEW 无人机：追击最近的 EW 无人机（免疫干扰），并从已被拦截机认领的 EW 中排除
    for aew in anti_ew_drones_list:
        candidates = [ew for ew in ew_drones if id(ew) not in claimed_ew_ids]
        if aew.is_alive() and candidates:
            nearest_ew = min(candidates,
                             key=lambda ew: sum((p - q) ** 2 for p, q in zip(aew.position, ew.position)))
            claimed_ew_ids.add(id(nearest_ew))
            aew._assigned_target = nearest_ew
            chase_target(aew, nearest_ew, map_grid=map_grid)

    # 筛选防守方"可见"的进攻方（屏蔽扇区内不可见，反辐射自身不参与分配）
    visible_offensive = [
        d for d in offensive
        if not isinstance(d, AntiRadiationDrone)
        and _is_visible_to_defense(d, base_positions, jammers, defenders=defensive)
    ]

    # 拍卖候选：可见目标 + 各防守方仍在锁定期内的目标（短暂被屏蔽时继续追）
    auction_targets = list(visible_offensive)
    _seen_targets = {id(t) for t in auction_targets}
    for _d in free_defenders:
        _locked = getattr(_d, "_locked_target", None)
        if (_locked is not None
                and getattr(_locked, "is_alive", lambda: False)()
                and (current_time - getattr(_d, "_lock_start", current_time)) <= LOCK_TTL
                and id(_locked) not in _seen_targets):
            auction_targets.append(_locked)
            _seen_targets.add(id(_locked))
    # 基地近程威胁也进入候选：即使全被 ARD 屏蔽、且还没有任何锁定，也要触发终端防御
    if base_positions:
        for _a in offensive:
            if (not isinstance(_a, AntiRadiationDrone) and _a.is_alive()
                    and id(_a) not in _seen_targets
                    and min(distance(_a.position, bp) for bp in base_positions) <= BASE_GUARD_RADIUS):
                auction_targets.append(_a)
                _seen_targets.add(id(_a))

    # 拦截机优先分配 EW 无人机（干扰加权）
    if auction_targets and free_defenders:
        if assignment_cfg is None:
            if _auction_assign is not None:
                try:
                    assignment_cfg = _auction_assign.ImprovedAuctionConfig()
                except Exception:
                    assignment_cfg = None
            else:
                assignment_cfg = None
        # 将基地位置提供给拍卖算法以用于距离归一化（使用模块缓存）
        try:
            if _auction_assign is not None:
                _auction_assign.set_base_positions(base_positions if base_positions else None)
        except Exception:
            pass
        # 构建 jammed_mask 传入拍卖（双重保险：即使 jammed defender 误入 free_defenders）
        import numpy as np
        _jammed = np.zeros(len(free_defenders), dtype=bool)
        for _i, _d in enumerate(free_defenders):
            if getattr(_d, '_ew_jammed', False):
                _jammed[_i] = True
        try:
            if _auction_assign is not None and hasattr(_auction_assign, 'greedy_assignment'):
                assignment = _auction_assign.greedy_assignment(
                    free_defenders,
                    auction_targets,
                    config=assignment_cfg,
                    map_grid=map_grid,
                    jammed_mask=_jammed if _jammed.any() else None,
                )
            else:
                assignment = {d.name: None for d in free_defenders}
        except Exception:
            assignment = {d.name: None for d in free_defenders}

        # 锁定优先：短暂不可见/拍卖换目标时仍按承诺追击（受 LOCK_TTL 限制）
        for _d in free_defenders:
            _locked = getattr(_d, "_locked_target", None)
            if _locked is None or not getattr(_locked, "is_alive", lambda: False)():
                continue
            if current_time - getattr(_d, "_lock_start", current_time) <= LOCK_TTL:
                assignment[_d.name] = _locked

        # 终端防御：逼近任一基地的进攻方优先级最高，强制最近的防守方转向拦截。
        # 这能压过普通锁定，专门处理"防守方都在远处追目标、基地被漏网"的情况。
        try:
            base_list = base_positions or []
            if base_list:
                threats = []
                for _a in offensive:
                    if isinstance(_a, AntiRadiationDrone) or not _a.is_alive():
                        continue
                    _bd = min(distance(_a.position, bp) for bp in base_list)
                    if _bd <= BASE_GUARD_RADIUS:
                        threats.append((_bd, _a))
                threats.sort(key=lambda item: item[0])

                def _near_base(obj) -> bool:
                    return min(distance(obj.position, bp) for bp in base_list) <= BASE_GUARD_RADIUS

                # 每个威胁最多保留一个已在拦截它的防守方；多出来的防守方可以去接新威胁，
                # 否则所有防守方都锁在旧威胁上，新威胁会没人管。
                _threat_by_id = {id(a): a for _, a in threats}
                _guard_candidates = {}
                for _d in free_defenders:
                    _t = getattr(_d, "_locked_target", None)
                    if (_t is not None and getattr(_t, "is_alive", lambda: False)()
                            and id(_t) in _threat_by_id and _near_base(_t)):
                        _guard_candidates.setdefault(id(_t), []).append(_d)
                _guarded_ids = set()
                _used_guards = set()
                for _tid, _guards in _guard_candidates.items():
                    _target = _threat_by_id[_tid]
                    _chosen = min(_guards, key=lambda d: distance(d.position, _target.position))
                    _guarded_ids.add(_tid)
                    _used_guards.add(id(_chosen))
                threats = [(bd, a) for bd, a in threats if id(a) not in _guarded_ids]
                _available = [
                    d for d in free_defenders
                    if d.is_alive() and id(d) not in _used_guards
                ]
                for _bd, _threat in threats:
                    if not _available:
                        break
                    _guard = min(_available, key=lambda d: distance(d.position, _threat.position))
                    _available.remove(_guard)
                    assignment[_guard.name] = _threat
                    _guard._locked_target = _threat
                    _guard._lock_start = current_time
                    _guard._locked_time = current_time
        except Exception:
            pass

        # 同一目标最多保留一架防守方：锁定/终端防御可能让多架压到同一个目标上，
        # 表现在画面里就是两架无人机贴在一起反复抢位置。保留离目标最近的一架。
        _claimed = {}
        for _d in free_defenders:
            _t = assignment.get(_d.name)
            if _t is None:
                continue
            _key = getattr(_t, "name", id(_t))
            _holder = _claimed.get(_key)
            if _holder is None:
                _claimed[_key] = _d
                continue
            if distance(_d.position, _t.position) < distance(_holder.position, _t.position):
                assignment[_holder.name] = None
                _holder._locked_target = None
                _holder._lock_start = None
                _claimed[_key] = _d
            else:
                assignment[_d.name] = None
                _d._locked_target = None
                _d._lock_start = None

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
                    defender._path_follower = None
                    chase_target(defender, target)
            else:
                try:
                    defender._assigned_target = None
                except Exception:
                    pass
                defender._path_follower = None
                defender.set_velocity([0.0, 0.0, 0.0])

        if map_grid is not None and pairs:
            # 用“防守方 -> 目标”的签名判断缓存路径是否仍然对应；只比长度会在
            # 分配互换但数量不变时把 A 的路径套到 B 身上。
            signature = tuple(
                (d.name, getattr(getattr(d, "_assigned_target", None), "name", None))
                for d in defenders_order
            )
            paths_updated = False
            plans = update_chase_strategy._last_cbs_paths
            need_replan = (
                (current_time - update_chase_strategy._last_cbs_time) >= CBS_REPLAN_INTERVAL
                or signature != update_chase_strategy._last_cbs_signature
            )
            if not need_replan and plans is None:
                need_replan = True

            if need_replan:
                try:
                    plan_inputs = [(p[0], p[1]) for p in pairs]
                    tick_seconds = (float(map_grid.cell_size) / plan_speed
                                    if plan_speed > 0 else 0.5)
                    if _cbs_pathplan is not None and hasattr(_cbs_pathplan, 'cbs_plan_schedules'):
                        plans = _cbs_pathplan.cbs_plan_schedules(
                            plan_inputs, map_grid=map_grid, tick_seconds=tick_seconds,
                            max_agents=10)
                    else:
                        plans = []
                except Exception:
                    plans = []
                update_chase_strategy._last_cbs_time = current_time
                update_chase_strategy._last_cbs_paths = plans
                update_chase_strategy._last_cbs_signature = signature
                paths_updated = True

            if plans:
                for defender, plan in zip(defenders_order, plans):
                    target_obj = assignment.get(defender.name)
                    if not plan:
                        defender._path_follower = None
                        if target_obj is not None:
                            chase_target(defender, target_obj)
                        continue

                    points, times = plan
                    follower = getattr(defender, "_path_follower", None)
                    if paths_updated:
                        if follower is None:
                            follower = PathFollower(defender, plan_speed=plan_speed)
                            defender._path_follower = follower
                        follower.set_path(points, current_time, times=times)
                        try:
                            defender._cbs_path = points
                            defender._cbs_goal = points[-1]
                        except Exception:
                            pass
                    elif follower is None:
                        # 缓存路径没有覆盖这个新分配的防守方：等下一次重规划
                        if target_obj is not None:
                            chase_target(defender, target_obj)
                    # 路径推进统一放在 update_path_followers 里，每帧执行
            else:
                # 规划失败或无路径，退化为直接追逐
                for defender in defenders_order:
                    defender._path_follower = None
                    target_obj = assignment.get(defender.name)
                    if target_obj is not None:
                        chase_target(defender, target_obj)
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

    # 全局去重：常规分配、被 EW 干扰的拦截机、AntiEW 可能从不同路径选中同一目标。
    # 同一目标只保留离它最近的一架，避免两架同时贴脸/原地抽搐。
    _claims = {}
    for d in defensive:
        if not d.is_alive():
            continue
        t = getattr(d, "_assigned_target", None)
        if t is None or not getattr(t, "is_alive", lambda: False)():
            continue
        key = getattr(t, "name", id(t))
        holder = _claims.get(key)
        if holder is None:
            _claims[key] = d
            continue
        if distance(d.position, t.position) < distance(holder.position, t.position):
            loser = holder
            _claims[key] = d
        else:
            loser = d
        loser._assigned_target = None
        loser._path_follower = None
        loser._locked_target = None
        loser._lock_start = None
        loser.set_velocity([0.0, 0.0, 0.0])


#=================暂行无人机生成策略，之后要被任务分配算法替代============================================

def spawn_random_drone(config: dict, drones: list, base_positions: list | None = None,
                       map_grid=None) -> None:
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

    def _edge_position():
        # 生成点保持在可视地图/规划网格内部（±500），避免在边界外被建筑 guard 卡住
        low, high = 460.0, 490.0
        side = 480.0
        direction = random.choice(['top', 'bottom', 'left', 'right'])
        if direction == 'top':
            return random.uniform(-side, side), random.uniform(low, high)
        if direction == 'bottom':
            return random.uniform(-side, side), random.uniform(-high, -low)
        if direction == 'left':
            return random.uniform(-high, -low), random.uniform(-side, side)
        return random.uniform(low, high), random.uniform(-side, side)

    def _spawn_position(x, y, z):
        # 进攻方若恰好刷在建筑 footprint 内，直接抬到楼顶+10 m，避免出生即贴墙/卡住
        if map_grid is not None and hasattr(map_grid, "get_max_height_at_world"):
            try:
                roof = float(map_grid.get_max_height_at_world(x, y))
                z = max(float(z), roof + 10.0)
            except Exception:
                pass
        return [float(x), float(y), float(min(350.0, z))]

    # 反辐射无人机
    if roll < ard_prob:
        z = random.uniform(100.0, 250.0)
        x, y = _edge_position()
        position = _spawn_position(x, y, z)
        name = f"AntiRad-{random.randint(100, 999)}"
        create_anti_radiation_drone(name, position, config, drones)
        LOGGER.info("Spawned AntiRadiation drone: %s at %s", name, position)
    elif roll < ard_prob + ew_prob:
        # 电子战干扰无人机：从地图边界生成
        z = random.uniform(100.0, 250.0)
        x, y = _edge_position()
        position = _spawn_position(x, y, z)
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
                # 扇区方向可能指向地图外；夹到可视地图内，避免出生在边界外
                x = max(-480.0, min(480.0, x))
                y = max(-480.0, min(480.0, y))
                position = _spawn_position(x, y, z)
                name = f"{drone_type.capitalize()}-{random.randint(100, 999)}"
                if drone_type == "attack":
                    create_attack_drone(name, position, config, drones)
                elif drone_type == "tank":
                    create_tank_drone(name, position, config, drones)
                return

        z = random.uniform(100.0, 250.0)
        x, y = _edge_position()
        position = _spawn_position(x, y, z)
        name = f"{drone_type.capitalize()}-{random.randint(100, 999)}"
        if drone_type == "attack":
            create_attack_drone(name, position, config, drones)
        elif drone_type == "tank":
            create_tank_drone(name, position, config, drones)

def balance_defenders(config: dict, drones: list, base_positions: list | None = None,
                      map_grid=None) -> None:
    """基于进攻方数量补充防守方（保持约 1:1）。

    统计全部非 ARD 进攻方，保证防守方数量不因 ARD 屏蔽而严重不足；
    ARD 只影响“能不能分配到目标”，不影响编制。新防守方随机从基地附近起飞，
    不再堆在原点，也避免全部挤在同一个基地。
    """
    offensive = [d for d in drones if _is_offensive(d) and d.is_alive()]
    # 统计所有存活、非 AntiEW 的防守方；被 EW 干扰的也要算编制，否则会重复补充
    from drones.defensive.anti_ew_drone import AntiEWDrone

    defenders_alive = [
        d for d in drones
        if not _is_offensive(d)
        and d.is_alive()
        and not isinstance(d, AntiEWDrone)
    ]
    target_count = int(
        len([a for a in offensive if not isinstance(a, AntiRadiationDrone)])
        * float(config.get("simulation", {}).get("defender_ratio", 1.0))
    )
    need = max(0, target_count - len(defenders_alive))
    if need <= 0:
        return

    fallback_base = config.get("base", {}).get("position", [0.0, 0.0, 0.0])
    interceptor_ratio = float(config.get("simulation", {}).get("defender_interceptor_ratio", 0.5))

    def _free_spawn(base) -> list:
        # 先在基地外围环形找没有被建筑占据的点
        for _ in range(24):
            angle = random.uniform(0.0, 2.0 * math.pi)
            radius = random.uniform(15.0, 45.0)
            x = float(base[0]) + radius * math.cos(angle)
            y = float(base[1]) + radius * math.sin(angle)
            if map_grid is None or not map_grid.is_occupied(x, y):
                return [x, y, random.uniform(30.0, 80.0)]
        # 环形都被占用时，在基地周围网格里找一个空格
        if map_grid is not None:
            bgx, bgy = map_grid.world_to_grid(base[0], base[1])
            for _ in range(160):
                gx = bgx + random.randint(-15, 15)
                gy = bgy + random.randint(-15, 15)
                if not map_grid.is_grid_occupied(gx, gy):
                    x, y = map_grid.grid_to_world(gx, gy)
                    return [float(x), float(y), random.uniform(30.0, 80.0)]
            # 基地周围都占满时，退到全图任意空格，绝不再生成到建筑里
            for _ in range(300):
                gx = random.randrange(map_grid.width)
                gy = random.randrange(map_grid.height)
                if not map_grid.is_grid_occupied(gx, gy):
                    x, y = map_grid.grid_to_world(gx, gy)
                    return [float(x), float(y), random.uniform(30.0, 80.0)]
        # 最后退回基地中心（正常不会走到这里）
        return [float(base[0]), float(base[1]), random.uniform(30.0, 80.0)]

    for _ in range(need):
        base = random.choice(base_positions) if base_positions else fallback_base
        drone_type = "interceptor" if random.random() < interceptor_ratio else "scout"
        position = _free_spawn(base)
        name = f"{drone_type.capitalize()}-{random.randint(100,999)}"
        if drone_type == "scout":
            create_scout_drone(name, position, config, drones)
        else:
            create_interceptor_drone(name, position, config, drones)


def _build_assignment_config(config: dict):
    """从主配置中构建任务分配配置，支持在 config.yaml 中直接调参。"""
    # 使用模块缓存以避免重复导入开销
    ImprovedAuctionConfig = None
    if _auction_assign is not None and hasattr(_auction_assign, 'ImprovedAuctionConfig'):
        ImprovedAuctionConfig = _auction_assign.ImprovedAuctionConfig

    raw = config.get("assignment", {})
    if not isinstance(raw, dict):
        LOGGER.warning("Config key 'assignment' must be a mapping. Using defaults.")
        return ImprovedAuctionConfig() if ImprovedAuctionConfig is not None else None

    if ImprovedAuctionConfig is None:
        try:
            import algorithms.auction_assign as _aa
            ImprovedAuctionConfig = _aa.ImprovedAuctionConfig
        except Exception:
            LOGGER.warning("auction_assign module not available, assignment config disabled")
            return None

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
    collision_count = 0
    #创建无人机初始团队
    base_manager = BaseManager(config)
    #创建基地管理器实例, 从配置中读取基地的相关参数
    # 雷达传感器已移除，仿真不再执行探测扫描
    
    # ---------- Open3D 可视化（失败则自动无界面运行）----------
    display = None
    enable_visualization = bool(config.get("simulation", {}).get("enable_visualization", True))
    if Open3DDisplay is not None and enable_visualization:
        try:
            obj_model_path = str(Path(__file__).resolve().parent / "Data" / "maps" / "3d66.com_JJI54558918189.obj")
            map_size = (config["map"]["width"], config["map"]["height"])
            use_obj = bool(config.get("map", {}).get("use_obj_model", True))
            enable_trails = bool(config.get("simulation", {}).get("enable_trails", True))
            if use_obj and os.path.exists(obj_model_path):
                display = Open3DDisplay(map_size=map_size, obj_model_path=obj_model_path,
                                        enable_trails=enable_trails, enable_dashed_lines=True)
            else:
                display = Open3DDisplay(map_size=map_size,
                                        enable_trails=enable_trails, enable_dashed_lines=True)
            display._init_map_data(config)
            display.open_window()
            LOGGER.info("Open3D window opened (R reset view, P pause)")
        except Exception as exc:
            LOGGER.warning("Open3D initialization failed, running headless: %s", exc)
            if display is not None:
                try:
                    display.close_window()
                except Exception:
                    pass
            display = None
    # 生成一次规划网格，仿真期间复用；有可视化时优先用可视化的地图，保证路径与建筑一致
    map_grid = getattr(display, "map_grid", None) if display is not None else None
    if map_grid is None:
        map_grid = _build_map_grid(config)
    update_chase_strategy._last_cbs_time = -1e9
    update_chase_strategy._last_cbs_paths = None
    update_chase_strategy._last_cbs_signature = None

    fps = config["simulation"]["fps"]
    frame_time = 1.0 / fps
    start_time = time.time()
    sim_time = 0.0
    plan_speed = float(config.get("path_planning", {}).get("plan_speed", 30.0))
    strategy_interval_s = float(config.get("simulation", {}).get("strategy_interval_s", 0.12))
    battery_drain_rate = float(config.get("simulation", {}).get("battery_drain_rate", 5.0))
    min_flight_altitude = float(config.get("simulation", {}).get("min_flight_altitude", 5.0))
    next_strategy_time = 0.0
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

    def _hud_metrics() -> dict:
        """HUD 用的攻防对应指标（与算法使用同一套可见性判断）。"""
        offensive_now = [d for d in drones if _is_offensive(d) and d.is_alive()]
        defensive_now = [d for d in drones if not _is_offensive(d) and d.is_alive()]
        jammers_now = _get_jammers(drones)
        visible_count = sum(
            1 for a in offensive_now
            if not isinstance(a, AntiRadiationDrone)
            and _is_visible_to_defense(a, base_positions, jammers_now, defenders=defensive_now)
        )
        assigned_count = sum(
            1 for d in defensive_now if getattr(d, "_assigned_target", None) is not None
        )
        return {"visible": visible_count, "assigned": assigned_count,
                "collisions": collision_count}

    # 将基地位置传递给拍卖算法以便距离归一化（使用模块缓存）
    try:
        if _auction_assign is not None:
            _auction_assign.set_base_positions(base_positions)
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
            if hasattr(drone, '_path_follower'):
                drone._path_follower = None
            # 分配目标标记
            if hasattr(drone, '_assigned_target'):
                drone._assigned_target = None
        # 重置全局 CBS 缓存
        if hasattr(update_chase_strategy, '_last_cbs_time'):
            update_chase_strategy._last_cbs_time = -1e9
        if hasattr(update_chase_strategy, '_last_cbs_paths'):
            update_chase_strategy._last_cbs_paths = None
        update_chase_strategy._last_cbs_signature = None
        LOGGER.info("Reset completed")
    # ------------------------------------------

    # ---------- 添加无人机数量统计计时器 ----------
    stats_timer = 0.0
    stats_interval = 1.0  # 每秒打印一次
    # ------------------------------------------

    frame_counter = 0
    total_frame_time = 0.0
    end_reason = "loop_started"
    print("=== AirCombat simulation started (Ctrl+C or close window to stop) ===", flush=True)
    try:
        # 无限模式不再因为基地全毁而退出；由循环内的同进程重置继续跑
        while duration <= 0 or time.time() - start_time < duration:
            frame_start = time.perf_counter()

            # 暂停：只刷新窗口/特效/HUD，不推进仿真
            if display is not None and display.paused:
                total_pct = 100.0 * base_manager.get_health() / max(base_manager.get_max_health(), 1e-6)
                try:
                    display.update(drones, [], total_pct, base_positions, sim_time,
                                   metrics=_hud_metrics())
                    if not display.is_open:
                        end_reason = "visualization window closed"
                        break
                    if not display.app.run_one_tick():
                        end_reason = "GUI event loop ended"
                        break
                except Exception:
                    LOGGER.exception("Display update failed; switching to headless")
                    try:
                        display.close_window()
                    except Exception:
                        pass
                    display = None
                    time.sleep(frame_time)
                    continue
                time.sleep(frame_time)
                continue

            sim_time += frame_time
            
            #1.碰撞检测与处理
            t1 = time.perf_counter()
            collisions = detect_collisions(drones, now=sim_time)
            if collisions:
                resolve_collisions(collisions, now=sim_time)
                collision_count += len(collisions)
            t_collision = time.perf_counter() - t1

            #2. 更新每架无人机的电量, 如果电量耗尽则触发坠毁流程
            t1 = time.perf_counter()
            for drone in drones:
                drone.update_battery(frame_time, drain_rate=battery_drain_rate)
            t_battery = time.perf_counter() - t1

            #3. 更新坠毁无人机的状态
            for drone in drones:
                if drone.destroyed:
                    if drone.falling:
                        drone.update_fall(frame_time)
                    elif drone.impact:
                        drone.update_death_timer(frame_time)

            #4.剔除爆炸结束的无人机，将其从整个仿真中清除
            drones = [d for d in drones if not d.should_remove()]

            #5.获取存活无人机列表
            alive_drones = [d for d in drones if not d.destroyed]

            #5.5 反辐射无人机自毁检查（每 0.5 秒集中检查一次）
            ard_check_timer += frame_time
            if ard_check_timer >= ard_check_interval:
                ard_check_timer = 0.0
                for drone in alive_drones:
                    if isinstance(drone, AntiRadiationDrone) and drone.is_alive():
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

            #6. 更新存活无人机的追逐策略（大编队时降频；路径推进仍每帧执行）
            t1 = time.perf_counter()
            if sim_time >= next_strategy_time:
                update_chase_strategy(
                    alive_drones,
                    base_positions,
                    _ASSIGNMENT_CONFIG,
                    map_grid=map_grid,
                    base_positions=base_positions,
                    sim_time=sim_time,
                    plan_speed=plan_speed,
                )
                strategy_dt = time.perf_counter() - t1
                # 策略耗时越长，下次间隔越大：把开销摊到多帧，避免整局掉帧
                next_strategy_time = sim_time + max(
                    strategy_interval_s, min(1.0, strategy_dt * 3.0))
            update_path_followers(alive_drones, sim_time)
            t_chase = time.perf_counter() - t1

            #6.5 检测 EW 无人机，生成 AntiEW 应对
            _spawn_anti_ew_if_needed(config, alive_drones, base_positions,
                                     map_grid=map_grid)

            #7. 更新存活无人机的位置
            t1 = time.perf_counter()
            for drone in alive_drones:
                move_drone(drone, frame_time, map_grid=map_grid,
                           min_altitude=min_flight_altitude)
            t_move = time.perf_counter() - t1

            # 7.5 自动销毁超出地图边界两倍的无人机
            map_limit = 1000
            for drone in alive_drones:
                if abs(drone.position[0]) > map_limit or abs(drone.position[1]) > map_limit:
                    LOGGER.info(f"{drone.name} out of bounds at {drone.position}, destroyed.")
                    drone.death_cause = "out_of_bounds"
                    drone.apply_damage(drone.health)

            #8. 基地碰撞检测
            t1 = time.perf_counter()
            if base_manager.check_collisions(drones):
                if duration <= 0:
                    # 无限模式：同一进程内开新一局，避免进程退出后被外部重新启动出新窗口
                    LOGGER.info("All bases destroyed; resetting battle in-process.")
                    for base in base_manager.bases:
                        base["health"] = float(base.get("max_health", base.get("health", 100.0)))
                    drones.clear()
                    update_chase_strategy._last_cbs_time = -1e9
                    update_chase_strategy._last_cbs_paths = None
                    update_chase_strategy._last_cbs_signature = None
                    next_strategy_time = 0.0
                    next_spawn_time = random.uniform(spawn_min, spawn_max)
                    spawn_timer = 0.0
                    stats_timer = 0.0
                    sim_time = 0.0
                    collision_count = 0
                    continue
                LOGGER.info("Base destroyed! Attackers win!")
                end_reason = "base destroyed (finite duration mode)"
                break
            t_base_collision = time.perf_counter() - t1

            #9. 随机生成进攻无人机
            spawn_timer += frame_time
            if spawn_timer >= next_spawn_time:
                spawn_timer = 0.0
                next_spawn_time = random.uniform(spawn_min, spawn_max)
                if random.random() < 0.9:
                    spawn_random_drone(config, drones, base_positions, map_grid=map_grid)

            #10. 平衡防守方数量
            balance_defenders(config, drones, base_positions, map_grid=map_grid)

            # ---------- 新增：每秒打印无人机数量统计 ----------
            stats_timer += frame_time
            if stats_timer >= stats_interval:
                stats_timer = 0.0
                # 统计当前各种类型的无人机数量
                total = len(drones)
                alive = len([d for d in drones if d.is_alive()])
                offensive = len([d for d in drones if _is_offensive(d) and d.is_alive()])
                defensive = len([d for d in drones if not _is_offensive(d) and d.is_alive()])
                
                # 详细分类统计
                attack_count = len([d for d in drones if isinstance(d, AttackDrone) and d.is_alive()])
                tank_count = len([d for d in drones if isinstance(d, TankDrone) and d.is_alive()])
                ard_count = len([d for d in drones if isinstance(d, AntiRadiationDrone) and d.is_alive()])
                ew_count = len([d for d in drones if isinstance(d, EWDrone) and d.is_alive()])
                scout_count = len([d for d in drones if isinstance(d, ScoutDrone) and d.is_alive()])
                interceptor_count = len([d for d in drones if isinstance(d, InterceptorDrone) and d.is_alive()])
                anti_ew_count = len([d for d in drones if isinstance(d, AntiEWDrone) and d.is_alive()])
                
                LOGGER.info(
                    f"📊 Drone Stats: Total={total} | Alive={alive} | "
                    f"Offensive={offensive} (Attack={attack_count}, Tank={tank_count}, ARD={ard_count}, EW={ew_count}) | "
                    f"Defensive={defensive} (Scout={scout_count}, Interceptor={interceptor_count}, AntiEW={anti_ew_count})"
                )
            # ------------------------------------------------

            detections = []
            t1 = time.perf_counter()
            LOGGER.debug("Detected %d objects", len(detections))
            if display is not None:
                # HUD 需要 0~100 的总血量百分比
                total_pct = 100.0 * base_manager.get_health() / max(base_manager.get_max_health(), 1e-6)
                try:
                    display.update(drones, detections, total_pct, base_positions, sim_time,
                                   metrics=_hud_metrics())
                    if not display.is_open:
                        LOGGER.info("Visualization window closed, ending simulation.")
                        end_reason = "visualization window closed"
                        break
                    if not display.app.run_one_tick():
                        LOGGER.info("GUI event loop ended.")
                        end_reason = "GUI event loop ended"
                        break
                except Exception:
                    LOGGER.exception("Display update failed; switching to headless")
                    try:
                        display.close_window()
                    except Exception:
                        pass
                    display = None
                    time.sleep(frame_time)
                    continue
            t_vis = time.perf_counter() - t1
    
            frame_end = time.perf_counter()
            frame_total = frame_end - frame_start

            # 慢帧诊断：50~200ms 只记 debug，超过 200ms 才 warning，避免大编队刷屏
            if frame_total > 0.05:
                msg = (
                    f"Slow frame: {frame_total*1000:.1f}ms | "
                    f"Collision:{t_collision*1000:.2f}ms Battery:{t_battery*1000:.2f}ms "
                    f"Chase:{t_chase*1000:.2f}ms Move:{t_move*1000:.2f}ms "
                    f"BaseCollision:{t_base_collision*1000:.2f}ms Vis:{t_vis*1000:.2f}ms"
                )
                if frame_total > 0.20:
                    LOGGER.warning(msg)
                else:
                    LOGGER.debug(msg)
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
    except KeyboardInterrupt:
        end_reason = "user interrupt (Ctrl+C)"
        raise
    except Exception:
        end_reason = "unhandled exception"
        raise
    finally:
        if display is not None:
            try:
                display.close_window()
            except Exception:
                pass
            time.sleep(0.2)  # 等待 Open3D 清理，避免退出时崩溃
        if end_reason == "loop_started" and duration > 0:
            end_reason = "duration reached"
        print(f"\n===== SIMULATION ENDED ({end_reason}) =====", flush=True)
        LOGGER.info("Simulation ended: %s", end_reason)
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
    try:
        run_simulation(config)
    except KeyboardInterrupt:
        print("\n[AirCombat] Stopped by user (Ctrl+C).", flush=True)
    except Exception:
        LOGGER.exception("AirCombat simulation crashed")
        print("\n[AirCombat] Simulation ended due to an unhandled error; see traceback above.",
              flush=True)
    finally:
        print("[AirCombat] Program exited.", flush=True)
        LOGGER.info("Simulation ended")