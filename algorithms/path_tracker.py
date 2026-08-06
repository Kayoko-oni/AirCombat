"""轻量路径跟踪器（PathTracker）

功能：
- 读取项目配置（config.yaml）中的路径规划相关参数（若存在），并提供默认值。
- 使用 algorithms.cbs_pathplan.a_star_plan_world 与 line_of_sight_world 进行规划与视线检测。
- 缓存路径、索引与重规划冷却，记录重规划日志。

接口：
- tracker = PathTracker(drone, config_path=None)
- next_wp = tracker.update(goal_world, map_grid)
  - 返回 None 表示目标点可直达（调用方应直接朝目标点追击）
  - 返回 [x,y,z] 表示下一步航点（调用方应将速度指向该点）

该类本身不控制无人机速度，仅决定下一步航点。
"""
from typing import Optional, Tuple, List
import time
import yaml
from pathlib import Path
from utils.logger import get_logger

logger = get_logger("PathTracker")

try:
    from algorithms.cbs_pathplan import a_star_plan_world, line_of_sight_world
except Exception:
    a_star_plan_world = None
    line_of_sight_world = None


class PathTracker:
    def __init__(self, drone, config_path: Optional[str] = None):
        self.drone = drone
        self.path: Optional[List[List[float]]] = None
        self.idx = 0
        self.goal: Optional[Tuple[float, float, float]] = None
        self.last_replan = 0.0
        self.climb_active = False
        self.climb_target_z: Optional[float] = None
        # 短时缓存：用于缓存最近一次的视线/最大高度判断，减少频繁调用昂贵的 map_grid 函数
        self._los_cache = {
            'pos': None,
            'goal': None,
            'maxh': 0.0,
            'los': False,
            'time': 0.0,
        }
        # load defaults
        self.replan_distance = 1.0  # 米，目标变化超过此距离将触发重规划
        self.replan_cooldown = 2.0  # 秒，最小重规划间隔
        self.max_plan_time_ms = 30  # 毫秒
        self.path_tolerance = 1.0   # 米，小于此认为到达路径点
        self.debug_log = False
        self.climb_clearance = 5.0  # 米，爬升时需高出障碍的安全高度
        # 尝试读取配置文件
        try:
            cfg_path = Path(config_path) if config_path else Path(__file__).resolve().parents[1] / "config.yaml"
            if cfg_path.exists():
                with cfg_path.open("r", encoding="utf-8") as fh:
                    cfg = yaml.safe_load(fh) or {}
                    pp = cfg.get("path_planning", {})
                    self.replan_distance = float(pp.get("replan_distance", self.replan_distance))
                    self.replan_cooldown = float(pp.get("replan_cooldown", self.replan_cooldown))
                    self.max_plan_time_ms = int(pp.get("max_plan_time_ms", self.max_plan_time_ms))
                    self.path_tolerance = float(pp.get("path_tolerance", self.path_tolerance))
                    self.debug_log = bool(pp.get("debug_log", self.debug_log))
                    self.climb_clearance = float(pp.get("climb_clearance", self.climb_clearance))
        except Exception as exc:
            logger.warning("Failed to load path planning config: %s", exc)

    def _dist2d(self, a, b):
        return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5

    def update(self, goal_world: Tuple[float, float, float], map_grid) -> Optional[List[float]]:
        """更新并返回下一步航点（世界坐标）；None 表示目标直达，调用方应直接追踪目标。"""
        if map_grid is None or a_star_plan_world is None or line_of_sight_world is None:
            return None
        now = time.time()
        goal = (float(goal_world[0]), float(goal_world[1]), float(goal_world[2] if len(goal_world) > 2 else 0.0))
        pos = tuple(self.drone.position[:3])

        # 如果正在爬升阶段，则优先继续爬升（这个分支很便宜）
        if self.climb_active:
            current_z = float(self.drone.position[2])
            if abs(current_z - (self.climb_target_z or 0.0)) < 1.0:
                self.climb_active = False
                self.climb_target_z = None
            else:
                return [float(pos[0]), float(pos[1]), float(self.climb_target_z)]

        # 判断是否需要重规划（目标变化超过阈值或无目标）
        need_replan = False
        if self.goal is None:
            need_replan = True
        else:
            if self._dist2d(self.goal, goal) > self.replan_distance:
                need_replan = True

        # 如果已有路径且未超出容差，则尝试推进索引
        if self.path is not None and not need_replan:
            while self.idx < len(self.path) and self._dist2d(pos, self.path[self.idx]) < self.path_tolerance:
                self.idx += 1
            if self.idx >= len(self.path):
                self.path = None
                self.goal = None
                return None
            return self.path[self.idx]

        # 限制重规划频率：如果在冷却时间内则直接使用旧路径（或退化为直接追击）
        if now - self.last_replan < self.replan_cooldown:
            if self.path is not None and self.idx < len(self.path):
                return self.path[self.idx]
            return None

        # 下面进行视线检查与最多一次的昂贵高度查询，使用短时缓存减少重复计算
        los = False
        maxh = 0.0
        try:
            cache = self._los_cache
            # 简单的网格化键（米级），对 pos/goal 做四舍五入以增加缓存命中
            key_pos = (round(pos[0], 0), round(pos[1], 0), round(pos[2], 0))
            key_goal = (round(goal[0], 0), round(goal[1], 0), round(goal[2], 0))
            if cache['pos'] == key_pos and cache['goal'] == key_goal and (now - cache['time']) < 0.5:
                los = cache['los']
                maxh = cache['maxh']
            else:
                # 计算视线可达性（可能昂贵）
                try:
                    los = line_of_sight_world(pos, goal, map_grid)
                except Exception:
                    los = False
                if hasattr(map_grid, "max_height_along_line"):
                    try:
                        maxh = map_grid.max_height_along_line(pos, goal)
                    except Exception:
                        maxh = 0.0
                # 更新缓存
                cache['pos'] = key_pos
                cache['goal'] = key_goal
                cache['los'] = los
                cache['maxh'] = maxh
                cache['time'] = now
        except Exception:
            los = False
            maxh = 0.0
        # 如果视线可达且高度安全，则直接追逐
        try:
            if pos[2] > maxh + self.climb_clearance and goal[2] > maxh + self.climb_clearance:
                self.path = None
                self.goal = None
                self.climb_active = False
                return None
        except Exception:
            pass

        if not los:
            # 若视线被阻挡且高度不足，则选择爬升避障并缓存该决定
            if pos[2] <= maxh + self.climb_clearance:
                climb_z = float(maxh + self.climb_clearance + 1.0)
                self.climb_active = True
                self.climb_target_z = climb_z
                try:
                    self.drone._avoid_path = [[float(pos[0]), float(pos[1]), float(pos[2])], [float(pos[0]), float(pos[1]), climb_z]]
                except Exception:
                    pass
                self.last_replan = now
                return [float(pos[0]), float(pos[1]), climb_z]

        # 执行 A* 规划（受限时长）
        t0 = time.time()
        plan = a_star_plan_world(pos, goal, map_grid, max_time_ms=self.max_plan_time_ms)
        elapsed_ms = (time.time() - t0) * 1000.0
        success = bool(plan)
        if success:
            if len(plan) <= 2:
                climb_z = max(pos[2], 20.0)
                # 直接提升高度并继续直追
                self.goal = goal
                self.last_replan = now
                if self.debug_log:
                    logger.debug("Path too short (len=%d), direct chase for %s", len(plan), self.drone.name)
                return [pos[0], pos[1], climb_z]

            self.path = plan
            self.idx = 1 if len(plan) > 1 else 0
            self.goal = goal
            self.last_replan = now
            try:
                self.drone._avoid_path = self.path
            except Exception:
                pass
            logger.info("Replan success: %s time=%.1fms len=%d", self.drone.name, elapsed_ms, len(plan))
            return self.path[self.idx] if self.idx < len(self.path) else None
        else:
            climb_z = max(pos[2], 20.0)
            return [pos[0], pos[1], climb_z]