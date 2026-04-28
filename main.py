import random
import time
from dataclasses import fields
from pathlib import Path

import yaml

from Controller.collision_handler import detect_collisions, resolve_collisions
from drones.offensive.attack_drone import AttackDrone
from drones.offensive.tank_drone import TankDrone
from drones.defensive.scout_drone import ScoutDrone
from drones.defensive.interceptor_drone import InterceptorDrone
from drones.factory import create_drone_team, create_attack_drone, create_tank_drone, create_interceptor_drone, create_scout_drone
from Controller.single_control import chase_target, chase_point, move_drone
from utils.logger import get_logger
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
    return drone.drone_type in {"AttackDrone", "TankDrone"}


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


def update_chase_strategy(drones, offensive_target_point, assignment_cfg=None, map_grid=None):
    """输入一个无人机实例列表, 让防守方追击根据任务分配得到的目标进攻方, 进攻方追击传入的offensive_target_point

    参数说明：
    - drones: 无人机实例列表
    - offensive_target_point: 进攻方的公共目标点
    - assignment_cfg: 任务分配配置
    - map_grid: 可选的 MapGrid 实例（来自 display.map_grid）。若提供则尝试使用 CBS 进行二维路径规划，输出为每架防守方的下一步航点。

    兼容性：当 map_grid 为 None 时，退化为原有的直接追逐逻辑。
    """
    offensive = [d for d in drones if _is_offensive(d) and d.is_alive()]
    defensive = [d for d in drones if not _is_offensive(d) and d.is_alive()]
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

    for drone in offensive:
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
                if line_of_sight_world is not None and not line_of_sight_world(pos, offensive_target_point, map_grid):
                    needs_plan = True
                if not needs_plan and _needs_obstacle_aware_path(pos, offensive_target_point, map_grid):
                    needs_plan = True
                # 如果不可直达或低空贴建筑，尝试做一个快速单体 A* 规划
                if needs_plan:
                    if a_star_plan_world is not None:
                        path = a_star_plan_world(pos, offensive_target_point, map_grid, max_time_ms=50)
                        if path:
                            try:
                                drone._avoid_path = path
                                drone._avoid_idx = 1 if len(path) > 1 else 0
                                next_wp = path[drone._avoid_idx]
                                # 直接追踪第一个航点（map_grid=None 以避免再次触发规划器）
                                chase_point(drone, next_wp, map_grid=None)
                                used_light_plan = True
                            except Exception:
                                used_light_plan = False
            except Exception:
                used_light_plan = False

        if not used_light_plan:
            chase_point(drone, offensive_target_point, map_grid=map_grid)
    #遍历进攻方无人机列表, 将每个无人机的目标设置为地图原点（基地）,
    #如果确认该目标存活, 则调用Controller——single_control——chase_target函数, 使其向目标无人机的位置运动

    if offensive and defensive:
        from algorithms.auction_assign import ImprovedAuctionConfig, greedy_assignment

        if assignment_cfg is None:
            assignment_cfg = ImprovedAuctionConfig()
        assignment = greedy_assignment(
            defensive,
            offensive,
            config=assignment_cfg,
            map_grid=map_grid,
        )  # 键是名字字符串
        # 构建名字到防守方对象的映射
        name_to_defender = {d.name: d for d in defensive}
        # 将 assignment 转为 (defender_obj, goal_pos) 列表供 CBS 使用
        pairs = []
        defenders_order = []
        for def_name, target in assignment.items():
            defender = name_to_defender.get(def_name)
            if defender is None:
                continue
            if target is not None:
                # 目标可以是一个无人机实例，取其当前世界位置作为规划目标
                # 记录分配给 defender 的目标对象，供可视化使用
                try:
                    defender._assigned_target = target
                except Exception:
                    pass
                goal_pos = target.position if hasattr(target, "position") else None
                if goal_pos is not None:
                    pairs.append((defender, goal_pos))
                    defenders_order.append(defender)
                else:
                    # 无法解析目标，退回到直接追踪
                    chase_target(defender, target)
            else:
                # 没有分配目标，清除标记并停在原地
                try:
                    defender._assigned_target = None
                except Exception:
                    pass
                defender.set_velocity([0.0, 0.0, 0.0])

        # 如果提供了地图，则调用 CBS 进行批量路径规划（返回世界坐标路径列表）
        if map_grid is not None and pairs:
            try:
                from algorithms.cbs_pathplan import cbs_plan_paths
                # 调用 cbs，输入为 (start_obj, goal_world_pos) 二元组列表
                plan_inputs = [(p[0], p[1]) for p in pairs]
                world_paths = cbs_plan_paths(plan_inputs, map_grid=map_grid)
            except Exception:
                world_paths = []

            if world_paths:
                # 对于每个防守方，取路径的第二个点（第一点通常是起点）作为下一步航点
                for defender, path in zip(defenders_order, world_paths):
                    if path and len(path) >= 2:
                        next_wp = path[1]
                        # next_wp 是 [x,y,z]
                        chase_point(defender, next_wp)
                        # 将整条路径缓存到 defender 上，供后续可能使用（非必需）
                        try:
                            defender._cbs_path = path
                            defender._cbs_goal = path[-1]
                        except Exception:
                            pass
                    elif path and len(path) == 1:
                        chase_point(defender, path[0])
                    else:
                        # 路径为空，退化为直接追踪目标对象
                        target_obj = assignment.get(defender.name)
                        if target_obj is not None:
                            chase_target(defender, target_obj)
            else:
                # CBS 未返回有效路径，退化为直接追踪
                for def_name, target in assignment.items():
                    defender = name_to_defender.get(def_name)
                    if defender is None:
                        continue
                    if target is not None:
                        chase_target(defender, target)
                    else:
                        defender.set_velocity([0.0, 0.0, 0.0])
        else:
            # 没有 map_grid，则保持原有行为：直接追踪
            for def_name, target in assignment.items():
                defender = name_to_defender.get(def_name)
                if defender is None:
                    continue
                if target is not None:
                    chase_target(defender, target)
                else:
                    defender.set_velocity([0.0, 0.0, 0.0])
    elif defensive:
        # 没有进攻方时，防守方停止移动
        for defender in defensive:
            defender.set_velocity([0.0, 0.0, 0.0])


#=================暂行无人机生成策略，之后要被任务分配算法替代============================================

def spawn_random_drone(config: dict, drones: list) -> None:
    #根据当前进攻方无人机数量随机生成1~2架新的进攻方无人机, 生成位置在地图左侧(-450~450, -400~400, 100), 类型随机为attack或tank
    offensive = [d for d in drones if _is_offensive(d) and d.is_alive()]
    sim_cfg = config.get("simulation", {})
    max_offensive = int(sim_cfg.get("max_offensive", 6))
    if len(offensive) >= max_offensive:
        return
    low_min = float(sim_cfg.get("offensive_spawn_z_low_min", 5.0))
    low_max = float(sim_cfg.get("offensive_spawn_z_low_max", 30.0))
    high_min = float(sim_cfg.get("offensive_spawn_z_high_min", 80.0))
    high_max = float(sim_cfg.get("offensive_spawn_z_high_max", 160.0))
    high_ratio = float(sim_cfg.get("offensive_spawn_high_ratio", 0.5))
    low_min, low_max = sorted((low_min, low_max))
    high_min, high_max = sorted((high_min, high_max))
    # 小规模仿真默认每次仅生成 1 架
    for _ in range(random.randint(1, 1)):
        drone_type = random.choice(["attack", "tank"])
        if drone_type in {"attack", "tank"}:
            if random.random() < max(0.0, min(1.0, high_ratio)):
                z = random.uniform(high_min, high_max)
            else:
                z = random.uniform(low_min, low_max)
            position = [random.uniform(-450, 450), random.uniform(-400, 400), z]
        name = f"{drone_type.capitalize()}-{random.randint(100,999)}"
        # 调用对应的生成函数，它们内部会执行 drones.append(drone)
        if drone_type == "attack":
            create_attack_drone(name, position, config, drones)
        elif drone_type == "tank":
            create_tank_drone(name, position, config, drones)
        # 注意：这里不需要再手动 drones.append()

def balance_defenders(config: dict, drones: list) -> None:
    """立即平衡防守方数量，直到防守方数量 >= 进攻方数量"""
    offensive = [d for d in drones if _is_offensive(d) and d.is_alive()]
    defensive = [d for d in drones if not _is_offensive(d) and d.is_alive()]
    # 计算需要补充的防守方数量
    need = max(0, len(offensive) - len(defensive))
    if need <= 0:
        return  #如果防守方数量大于进攻方，则直接返回
    # 一次性生成 need 架防守方（固定在基地）
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

    drones = create_drone_team(config)
    #创建无人机初始团队
    base_manager = BaseManager(config)
    #创建基地管理器实例, 从配置中读取基地的相关参数
    display = None
    if Open3DDisplay is not None:
        try:
            display = Open3DDisplay(map_size=(config["map"]["width"], config["map"]["height"]))
            display.open_window()
            # 提前初始化地图数据，使 display.map_grid 在仿真第一帧就可用
            try:
                display._init_map_data()
            except Exception:
                # 若初始化失败（应当不会），在后续 display.update() 时会再次尝试
                pass
        except Exception as exc:
            LOGGER.warning("Open3D initialization failed, running headless: %s", exc)
            display = None
    #尝试创建open3d窗口

    fps = config["simulation"]["fps"]
    frame_time = 1.0 / fps
    start_time = time.time()
    duration = config["simulation"]["duration"]
    # 生成间隔支持从配置读取，便于做小规模仿真
    spawn_min = config.get("simulation", {}).get("spawn_interval_min", 1.0)
    spawn_max = config.get("simulation", {}).get("spawn_interval_max", 3.0)
    next_spawn_time = random.uniform(spawn_min, spawn_max)
    spawn_timer = 0.0
    base_position = config["base"]["position"]   # 例如 [0, 0, 0]

    #将帧率fps的值设置为配置中simulation-fps的值
    #frame_time 为一帧所持续的时间
    #start_time 仿真开始的时间, 设置为从time.time()中获取的系统绝对时间
    #duration 仿真的持续时间, 从配置中的simulation-duration中获取
    #下一次无人机生成的时间, 在 1~3 之间生成一个随机数 (这里之后肯定是要改掉的)
    #进攻无人机生成时间的计时器初始化为0, 其值随着时间的进行同步增加, 当其大于next_spawn_time时生成新无人机(之后肯定要改掉的)


    try:
        while (duration <= 0 or time.time() - start_time < duration) and not base_manager.is_destroyed():
            # 当 duration<=0 时表示无限运行，否则按配置时长运行, 同时如果基地被摧毁了也要结束仿真

            #1.碰撞检测与处理
            collisions = detect_collisions(drones)  # 收集这一帧中发生碰撞的无人机的列表"""
            if collisions:
                resolve_collisions(collisions)  #如果在这一帧中发生了碰撞, 那么对发生碰撞的无人机对执行resolve_collisions中的碰撞处理函数"""

            #2. 更新每架无人机的电量, 如果电量耗尽则触发坠毁流程
            for drone in drones:
                drone.update_battery(frame_time)

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

            #5.获取存活无人机列表, 如果没有存活的无人机了, 输出到日志, 结束仿真
            alive_drones = [d for d in drones if not d.destroyed]
            if not alive_drones:
                LOGGER.info("All active drones destroyed, ending simulation.")
                break

            #6. 更新存活无人机的追逐策略
            # 将 display.map_grid 传入路径规划模块；若 display 不存在则传入 None
            update_chase_strategy(alive_drones, base_position, _ASSIGNMENT_CONFIG, map_grid=(display.map_grid if display is not None else None)) #进攻方的目标点暂时设定为基地位置
            # 更新每架无人机的追逐策略 """

            #7. 更新存活无人机的位置
            for drone in alive_drones:
                move_drone(drone, frame_time)
            #对在alive_drone列表中(也就是确认存活的)无人机, 执行位置更新操作"""

            #8. 基地碰撞检测
            if base_manager.check_collisions(drones):
                LOGGER.info("Base destroyed! Attackers win!")
                break

            #9. 随机生成进攻无人机
            spawn_timer += frame_time  #随机生成进攻方无人机
            if spawn_timer >= next_spawn_time:
                spawn_timer = 0.0
                next_spawn_time = random.uniform(spawn_min, spawn_max)
                if random.random() < 0.9:
                    spawn_random_drone(config, drones)

            #10. 平衡防守方数量
            balance_defenders(config, drones) #每帧立即平衡防守方数量, 直到防守方数量 >= 进攻方数量"""

            detections = []
            #雷达模块已移除，进攻方目标全局可知
            LOGGER.debug("Detected %d objects", len(detections))
            if display is not None:
                display.update(drones, detections, base_manager.get_health(), base_position)
                if not display.is_open:
                    LOGGER.info("Visualization window closed, ending simulation.")
                    break
                # Run GUI event loop tick
                if not display.app.run_one_tick():
                    LOGGER.info("GUI event loop ended.")
                    break
            time.sleep(frame_time)
    finally:
        if display is not None:
            display.close_window()
            #如果上面的try中发生错误, 则直接执行finally中的代码, 使仿真出错时可以正常退出程序"""



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
