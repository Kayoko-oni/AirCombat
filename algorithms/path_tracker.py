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
        # load defaults
        self.replan_distance = 1.0  # 米，目标变化超过此距离将触发重规划
        self.replan_cooldown = 0.5  # 秒，最小重规划间隔
        self.max_plan_time_ms = 50  # 毫秒
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

        # 先检查沿线最高建筑高度：若双方已经在安全高度之上则认为可直达
        maxh = 0.0
        try:
            if hasattr(map_grid, "max_height_along_line"):
                maxh = map_grid.max_height_along_line(pos, goal)
        except Exception:
            maxh = 0.0

        # 如果双方都在障碍之上并满足安全间隙，则认为视线可达
        try:
            if pos[2] > maxh + self.climb_clearance and goal[2] > maxh + self.climb_clearance:
                self.path = None
                self.goal = None
                self.climb_active = False
                return None
        except Exception:
            pass

        # 如果正在爬升阶段，则优先继续爬升
        if self.climb_active:
            current_z = float(self.drone.position[2])
            if abs(current_z - (self.climb_target_z or 0.0)) < 1.0:
                # 已达到爬升高度，结束爬升阶段，下一次循环会重试直接或规划路径
                self.climb_active = False
                self.climb_target_z = None
            else:
                # 继续爬升：返回垂直上升目标点
                return [float(pos[0]), float(pos[1]), float(self.climb_target_z)]

        need_replan = False
        if self.goal is None:
            need_replan = True
        else:
            # 目标变化超过阈值
            if self._dist2d(self.goal, goal) > self.replan_distance:
                need_replan = True

        # 如果已有路径且未超出容差，则尝试推进索引
        if self.path is not None and not need_replan:
            # 前进到下一个未到达点
            while self.idx < len(self.path) and self._dist2d(pos, self.path[self.idx]) < self.path_tolerance:
                self.idx += 1
            if self.idx >= len(self.path):
                # 已到达路径末尾
                self.path = None
                self.goal = None
                return None
            # 若路径仍有效，返回当前航点
            return self.path[self.idx]

        # 限制重规划频率
        if now - self.last_replan < self.replan_cooldown:
            # 如果有旧路径可用，继续使用，否则直接返回 None（直追）
            if self.path is not None and self.idx < len(self.path):
                return self.path[self.idx]
            return None

        # 如果视线被阻挡且高度不足，尝试触发爬升避障（简单处理）
        try:
            los = False
            try:
                los = line_of_sight_world(pos, goal, map_grid)
            except Exception:
                los = False

            if not los:
                maxh = 0.0
                if hasattr(map_grid, "max_height_along_line"):
                    maxh = map_grid.max_height_along_line(pos, goal)
                # 如果当前高度低于障碍顶端 + 安全间隙，则先爬升
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
        except Exception:
            # 任何异常都不应阻塞正常逻辑
            pass

        # 执行 A* 规划
        t0 = time.time()
        plan = a_star_plan_world(pos, goal, map_grid, max_time_ms=self.max_plan_time_ms)
        elapsed_ms = (time.time() - t0) * 1000.0
        success = bool(plan)
        if success:
            self.path = plan
            self.idx = 1 if len(plan) > 1 else 0
            self.goal = goal
            self.last_replan = now
            # 将路径缓存到无人机对象，便于可视化/调试
            try:
                self.drone._avoid_path = self.path
            except Exception:
                pass
            logger.info("Replan success: drone=%s time=%.1fms path_len=%d", getattr(self.drone, "name", "?"), elapsed_ms, len(plan))
            return self.path[self.idx] if self.idx < len(self.path) else None
        else:
            self.last_replan = now
            logger.info("Replan failed: drone=%s time=%.1fms", getattr(self.drone, "name", "?"), elapsed_ms)
            return None
*** End Patch