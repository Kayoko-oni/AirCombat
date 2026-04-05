import random
import time
from pathlib import Path

import yaml

from Controller.collision_handler import detect_collisions, resolve_collisions
from drones.offensive.attack_drone import AttackDrone
from drones.offensive.tank_drone import TankDrone
from drones.defensive.scout_drone import ScoutDrone
from drones.defensive.interceptor_drone import InterceptorDrone
from Controller.single_control import chase_target, move_drone
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


def create_drone_team(config: dict) -> list:
    """根据配置创建无人机团队"""
    return [
        AttackDrone(name="Attack-01", position=[-100, -50, 20], config=config["drones"]["attack"]),
        TankDrone(name="Tank-01", position=[-120, -50, -20], config=config["drones"]["tank"]),
        ScoutDrone(name="Scout-01", position=[100, 50, 30], config=config["drones"]["scout"]),
        InterceptorDrone(name="Intercepter-01", position=[120, 50, -30], config=config["drones"]["interceptor"]),
    ]


def _is_offensive(drone):
    return drone.drone_type in {"AttackDrone", "TankDrone"}


def _find_nearest_opponent(drone, candidates):
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


def update_chase_strategy(drones):
    offensive = [d for d in drones if _is_offensive(d) and d.is_alive()]
    defensive = [d for d in drones if not _is_offensive(d) and d.is_alive()]
    for drone in offensive:
        target = _find_nearest_opponent(drone, defensive)
        if target is not None:
            chase_target(drone, target)
    for drone in defensive:
        target = _find_nearest_opponent(drone, offensive)
        if target is not None:
            chase_target(drone, target)


def spawn_random_drone(config: dict, drones: list) -> None:
    active = [drone for drone in drones if not drone.destroyed]
    if len(active) >= 14:
        return
    for _ in range(random.randint(1, 2)):
        drone_type = random.choice(["attack", "tank", "scout", "interceptor"])
        config_key = drone_type
        if drone_type in {"attack", "tank"}:
            position = [random.uniform(-450, -250), random.uniform(-400, 400), random.uniform(-20, 40)]
        else:
            position = [random.uniform(250, 450), random.uniform(-400, 400), random.uniform(-20, 40)]
        # TODO: 使生成位置可配置而非硬编码
        if drone_type == "attack":
            drone = AttackDrone(name=f"Attack-{random.randint(100,999)}", position=position, config=config["drones"][config_key])
        elif drone_type == "tank":
            drone = TankDrone(name=f"Tank-{random.randint(100,999)}", position=position, config=config["drones"][config_key])
        elif drone_type == "scout":
            drone = ScoutDrone(name=f"Scout-{random.randint(100,999)}", position=position, config=config["drones"][config_key])
        else:
            drone = InterceptorDrone(name=f"Intercepter-{random.randint(100,999)}", position=position, config=config["drones"][config_key])
        drones.append(drone)


def run_simulation(config: dict):
    """运行仿真主循环"""
    drones = create_drone_team(config)
    radar = RadarSensor(range_m=config["radar"]["range"], pulse_interval=config["radar"]["pulse_interval"])
    display = None
    if Open3DDisplay is not None:
        try:
            display = Open3DDisplay(map_size=(config["map"]["width"], config["map"]["height"]))
            display.open_window()
        except Exception as exc:
            LOGGER.warning("Open3D initialization failed, running headless: %s", exc)
            display = None

    fps = config["simulation"]["fps"]
    frame_time = 1.0 / fps
    start_time = time.time()
    duration = config["simulation"]["duration"]
    next_spawn_time = random.uniform(1.0, 3.0)
    spawn_timer = 0.0

    try:
        while time.time() - start_time < duration:
            update_chase_strategy(drones)
            collisions = detect_collisions(drones)
            if collisions:
                resolve_collisions(collisions)

            for drone in drones:
                if drone.destroyed:
                    drone.update_death_timer(frame_time)

            drones = [d for d in drones if not d.should_remove()]
            alive_drones = [d for d in drones if not d.destroyed]
            if not alive_drones:
                LOGGER.info("All active drones destroyed, ending simulation.")
                break

            for drone in alive_drones:
                move_drone(drone, frame_time)

            spawn_timer += frame_time
            if spawn_timer >= next_spawn_time:
                spawn_timer = 0.0
                next_spawn_time = random.uniform(1.0, 3.0)
                if random.random() < 0.9:
                    spawn_random_drone(config, drones)

            detections = radar.scan(alive_drones)
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


if __name__ == "__main__":
    config_path = Path("config.yaml")
    config = load_config(config_path)
    LOGGER.info("Starting AirCombat simulation")
    run_simulation(config)
    LOGGER.info("Simulation ended")
