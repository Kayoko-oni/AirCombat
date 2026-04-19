import random
import time
from pathlib import Path

import yaml

from Controller.collision_handler import detect_collisions, resolve_collisions
from drones.offensive.attack_drone import AttackDrone
from drones.offensive.tank_drone import TankDrone
from drones.defensive.scout_drone import ScoutDrone
from drones.defensive.interceptor_drone import InterceptorDrone
from drones.factory import create_drone_team, create_attack_drone, create_tank_drone, create_interceptor_drone, create_scout_drone
from Controller.single_control import chase_target, chase_point, move_drone
from sensing.radar import RadarSensor
from utils.logger import get_logger

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


def update_chase_strategy(drones):
    """输入一个无人机实例列表, 让里面的所有无人机追逐距离自己最近的敌方无人机, 之后要被高阶算法替换"""
    offensive = [d for d in drones if _is_offensive(d) and d.is_alive()]
    defensive = [d for d in drones if not _is_offensive(d) and d.is_alive()]
    #先将无人机列表中的所有实例遍历, 将它们分为offensive和defensive两个列表
    for drone in offensive:
        chase_point(drone, [0.0, 0.0, 0.0])
    #遍历进攻方无人机列表, 将每个无人机的目标设置为地图原点（基地）,
    #如果确认该目标存活, 则调用Controller——single_control——chase_target函数, 使其向目标无人机的位置运动

    for drone in defensive:
        target = _find_nearest_opponent(drone, offensive)
        if target is not None:
            chase_target(drone, target)
    #遍历防守方无人机列表, 将每个无人机的目标设置为距离自己最近的敌机,
    #如果确认该目标存活, 则调用Controller——single_control——chase_target函数, 使其向目标无人机的位置运动


#=================暂行无人机生成策略，之后要被任务分配算法替代============================================

def spawn_random_drone(config: dict, drones: list) -> None:
    """ 场上进攻方数量小于7时, 随机生成类型随机的进攻无人机, 生成范围为地图的上界平面 """
    active = [d for d in drones if not d.destroyed]
    if len(active) >= 14:
        return
    for _ in range(random.randint(1, 2)):
        drone_type = random.choice(["attack", "tank"])
        if drone_type in {"attack", "tank"}:
            position = [random.uniform(-450, -250), random.uniform(-400, 400), 100]
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
    need = len(offensive) - len(defensive)
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

#==================================================================================================


def run_simulation(config: dict):
    """运行仿真主循环"""
    drones = create_drone_team(config)
    #创建无人机初始团队
    radar = RadarSensor(range_m=config["radar"]["range"], pulse_interval=config["radar"]["pulse_interval"])
    #创建雷达传感器实例, 从配置中读取range_m和pulse_interval
    display = None
    if Open3DDisplay is not None:
        try:
            display = Open3DDisplay(map_size=(config["map"]["width"], config["map"]["height"]))
            display.open_window()
        except Exception as exc:
            LOGGER.warning("Open3D initialization failed, running headless: %s", exc)
            display = None
    #尝试创建open3d窗口

    fps = config["simulation"]["fps"]
    frame_time = 1.0 / fps
    start_time = time.time()
    duration = config["simulation"]["duration"]
    next_spawn_time = random.uniform(1.0, 3.0)
    spawn_timer = 0.0

    #将帧率fps的值设置为配置中simulation-fps的值
    #frame_time 为一帧所持续的时间
    #start_time 仿真开始的时间, 设置为从time.time()中获取的系统绝对时间
    #duration 仿真的持续时间, 从配置中的simulation-duration中获取
    #下一次无人机生成的时间, 在 1~3 之间生成一个随机数 (这里之后肯定是要改掉的)
    #进攻无人机生成时间的计时器初始化为0, 其值随着时间的进行同步增加, 当其大于next_spawn_time时生成新无人机(之后肯定要改掉的)


    try:
        while time.time() - start_time < duration:
            #当仿真时间小于duration时, 继续进行仿真的新一帧 """
            update_chase_strategy(drones)
            # 更新每架无人机的追逐策略 """
            collisions = detect_collisions(drones)
            # 收集这一帧中发生碰撞的无人机的列表"""
            if collisions:
                resolve_collisions(collisions)
            #如果在这一帧中发生了碰撞, 那么对发生碰撞的无人机对执行 Controller——collision_handler——resolve_collisions中的碰撞处理函数"""

            for drone in drones:
                if drone.destroyed:
                    drone.update_death_timer(frame_time)
            #执行完碰撞处理函数后, 遍历所有无人机, 判断它们是否坠毁"""

            drones = [d for d in drones if not d.should_remove()]
            #剔除drones列表中所有被 .should_move函数判断为"应该被移除"的无人机, 即更新drone列表"""

            alive_drones = [d for d in drones if not d.destroyed]
            if not alive_drones:
                LOGGER.info("All active drones destroyed, ending simulation.")
                break
            #将所有未被摧毁的无人机添加至alive_drone列表, 如果没有存活的无人机了, 输出到日志, 结束仿真"""

            for drone in alive_drones:
                move_drone(drone, frame_time)
            #对在alive_drone列表中(也就是确认存活的)无人机, 执行位置更新操作"""

            spawn_timer += frame_time  #随机生成进攻方无人机
            if spawn_timer >= next_spawn_time:
                spawn_timer = 0.0
                next_spawn_time = random.uniform(1.0, 3.0)
                if random.random() < 0.9:
                    spawn_random_drone(config, drones)

            balance_defenders(config, drones) #根据进攻方无人机数量补充防守方无人机

            detections = radar.scan(alive_drones)
            #雷达扫描一次所有存活的无人机 """
            LOGGER.debug("Detected %d objects", len(detections))
            if display is not None:
                display.update(drones, detections)
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
  
    config_path = Path("config.yaml")
    config = load_config(config_path)
    LOGGER.info("Starting AirCombat simulation")
    run_simulation(config)
    LOGGER.info("Simulation ended")
