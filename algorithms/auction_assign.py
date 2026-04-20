from __future__ import annotations

import time
from dataclasses import dataclass
from importlib import import_module
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

try:
    # 优先复用 AirCombat 已定义的几何距离函数。
    _aircombat_distance = getattr(import_module("utils.geometry"), "distance")
except Exception:
    _aircombat_distance = None


@dataclass(frozen=True)
class AssignmentResult:
    """任务分配求解结果容器。"""

    task_to_agent: np.ndarray
    total_utility: float
    iterations: int
    runtime_seconds: float
    epsilon_schedule: List[float]

    def to_dict(self) -> dict:
        return {
            "task_to_agent": self.task_to_agent.tolist(),
            "total_utility": self.total_utility,
            "iterations": self.iterations,
            "runtime_seconds": self.runtime_seconds,
            "epsilon_schedule": self.epsilon_schedule,
        }


@dataclass
class ImprovedAuctionConfig:
    """改进拍卖算法配置参数。"""

    epsilon_start: float = 1.0
    epsilon_end: float = 1e-3
    epsilon_decay: float = 0.2
    max_iterations_per_phase: int = 20_000
    regret_weight: float = 0.1
    warm_start: bool = True
    warm_start_fraction: float = 0.35
    enable_local_refine: bool = True
    local_refine_rounds: int = 5


class ImprovedAuctionSolver:
    """改进拍卖算法（含 epsilon 缩放与局部交换精修）。

    目标：在一对一约束下最大化总效用。
    约束：每个任务必须分配给且仅分配给一个智能体；每个智能体最多执行一个任务。
    适用：支持 n_agents >= n_tasks 的非方阵输入。
    """

    def __init__(self, config: ImprovedAuctionConfig | None = None):
        self.config = config or ImprovedAuctionConfig()

    def solve(self, utility: np.ndarray) -> AssignmentResult:
        # 入口处统一做校验和类型归一化，避免后续重复判断。
        utility = self._validate_utility(utility)
        n_agents, n_tasks = utility.shape

        if n_agents < n_tasks:
            raise ValueError(
                "utility must satisfy n_agents >= n_tasks for one-to-one assignment"
            )

        started_at = time.perf_counter()
        square_utility, n_real_tasks = self._pad_to_square(utility)

        # owner_of_task[j] = i 表示任务 j 当前由智能体 i 持有。
        # task_of_agent[i] = j 表示智能体 i 当前持有任务 j。
        owner_of_task = np.full(n_agents, -1, dtype=np.int32)
        task_of_agent = np.full(n_agents, -1, dtype=np.int32)
        # 拍卖机制中的对偶价格。
        prices = np.zeros(n_agents, dtype=np.float64)

        if self.config.warm_start:
            self._greedy_warm_start(
                utility=square_utility,
                owner_of_task=owner_of_task,
                task_of_agent=task_of_agent,
                n_real_tasks=n_real_tasks,
            )

        epsilon_schedule = self._build_epsilon_schedule()
        total_iterations = 0

        # 从大 epsilon 到小 epsilon 分阶段求解，兼顾收敛速度与精度。
        for epsilon in epsilon_schedule:
            iters = self._auction_phase(
                utility=square_utility,
                owner_of_task=owner_of_task,
                task_of_agent=task_of_agent,
                prices=prices,
                epsilon=epsilon,
            )
            total_iterations += iters

        task_to_agent = owner_of_task[:n_real_tasks].copy()

        # 局部两两交换可修复竞价阶段可能遗留的次优匹配。
        if self.config.enable_local_refine:
            self._local_swap_refine(utility, task_to_agent, self.config.local_refine_rounds)

        total_utility = float(
            utility[task_to_agent, np.arange(n_real_tasks, dtype=np.int32)].sum()
        )

        runtime_seconds = time.perf_counter() - started_at
        return AssignmentResult(
            task_to_agent=task_to_agent,
            total_utility=total_utility,
            iterations=total_iterations,
            runtime_seconds=runtime_seconds,
            epsilon_schedule=epsilon_schedule,
        )

    def _validate_utility(self, utility: np.ndarray) -> np.ndarray:
        if not isinstance(utility, np.ndarray):
            utility = np.asarray(utility, dtype=np.float64)

        if utility.ndim != 2:
            raise ValueError("utility must be a 2D matrix")

        if utility.shape[0] == 0 or utility.shape[1] == 0:
            raise ValueError("utility matrix must be non-empty")

        if not np.isfinite(utility).all():
            raise ValueError("utility contains NaN or Inf values")

        return utility.astype(np.float64, copy=False)

    def _pad_to_square(self, utility: np.ndarray) -> Tuple[np.ndarray, int]:
        n_agents, n_tasks = utility.shape
        if n_agents == n_tasks:
            return utility, n_tasks

        # 用“虚拟任务”补齐方阵，便于统一拍卖流程。
        padded = np.zeros((n_agents, n_agents), dtype=np.float64)
        padded[:, :n_tasks] = utility
        return padded, n_tasks

    def _build_epsilon_schedule(self) -> list[float]:
        cfg = self.config
        if cfg.epsilon_start <= 0 or cfg.epsilon_end <= 0:
            raise ValueError("epsilon_start and epsilon_end must be positive")

        if cfg.epsilon_start < cfg.epsilon_end:
            raise ValueError("epsilon_start must be >= epsilon_end")

        if not (0 < cfg.epsilon_decay < 1):
            raise ValueError("epsilon_decay must be in (0, 1)")

        # 乘法降温：eps_{k+1} = max(eps_end, eps_k * decay)。
        schedule = [cfg.epsilon_start]
        while schedule[-1] > cfg.epsilon_end:
            next_eps = max(cfg.epsilon_end, schedule[-1] * cfg.epsilon_decay)
            if abs(next_eps - schedule[-1]) < 1e-12:
                break
            schedule.append(next_eps)

        if schedule[-1] != cfg.epsilon_end:
            schedule.append(cfg.epsilon_end)

        return schedule

    def _greedy_warm_start(
        self,
        utility: np.ndarray,
        owner_of_task: np.ndarray,
        task_of_agent: np.ndarray,
        n_real_tasks: int,
    ) -> None:
        if not (0.0 < self.config.warm_start_fraction <= 1.0):
            raise ValueError("warm_start_fraction must be in (0, 1]")

        if n_real_tasks <= 0:
            return

        target = int(np.ceil(n_real_tasks * self.config.warm_start_fraction))
        target = max(1, min(target, n_real_tasks))

        # 仅在真实任务上做部分热启动，保留后续全局优化空间。
        real_block = utility[:, :n_real_tasks]
        n_agents = utility.shape[0]
        pairs = np.dstack(
            np.unravel_index(np.argsort(real_block.ravel())[::-1], (n_agents, n_real_tasks))
        )[0]

        assigned_count = 0

        for agent_idx, task_idx in pairs:
            if task_of_agent[agent_idx] != -1:
                continue
            if owner_of_task[task_idx] != -1:
                continue

            task_of_agent[agent_idx] = task_idx
            owner_of_task[task_idx] = agent_idx

            assigned_count += 1
            if assigned_count >= target:
                break

    def _auction_phase(
        self,
        utility: np.ndarray,
        owner_of_task: np.ndarray,
        task_of_agent: np.ndarray,
        prices: np.ndarray,
        epsilon: float,
    ) -> int:
        n = utility.shape[0]
        iterations = 0

        for _ in range(self.config.max_iterations_per_phase):
            # 仅让当前未分配的智能体重新出价。
            unassigned_agents = np.flatnonzero(task_of_agent == -1)
            if unassigned_agents.size == 0:
                break

            for agent_idx in unassigned_agents:
                # 标准拍卖净值：效用减去当前价格。
                net_values = utility[agent_idx] - prices
                best_task = int(np.argmax(net_values))
                best_value = float(net_values[best_task])

                if n == 1:
                    second_best_value = best_value
                else:
                    # 用 partition 找次大值，通常比完整排序更快。
                    second_best_value = float(np.partition(net_values, -2)[-2])

                row = utility[agent_idx]
                # regret 项会增强“明显更优”候选任务的出价强度。
                regret = max(0.0, float(row[best_task] - row.mean()))
                bid_increment = (best_value - second_best_value) + epsilon * (
                    1.0 + self.config.regret_weight * regret
                )
                prices[best_task] += bid_increment

                prev_owner = int(owner_of_task[best_task])
                owner_of_task[best_task] = agent_idx
                task_of_agent[agent_idx] = best_task

                if prev_owner != -1:
                    # 被抢占任务的原持有者变为未分配，下一轮继续竞价。
                    task_of_agent[prev_owner] = -1

                iterations += 1

        if np.any(task_of_agent == -1):
            raise RuntimeError(
                "auction phase did not converge. Increase max_iterations_per_phase."
            )

        return iterations

    def _local_swap_refine(
        self,
        utility: np.ndarray,
        task_to_agent: np.ndarray,
        max_rounds: int,
    ) -> None:
        n_tasks = task_to_agent.size
        for _ in range(max_rounds):
            improved = False
            for task_a in range(n_tasks - 1):
                for task_b in range(task_a + 1, n_tasks):
                    agent_a = int(task_to_agent[task_a])
                    agent_b = int(task_to_agent[task_b])

                    current = utility[agent_a, task_a] + utility[agent_b, task_b]
                    swapped = utility[agent_a, task_b] + utility[agent_b, task_a]

                    if swapped > current + 1e-12:
                        task_to_agent[task_a], task_to_agent[task_b] = (
                            task_to_agent[task_b],
                            task_to_agent[task_a],
                        )
                        improved = True

            if not improved:
                break


def build_utility_from_positions(
    agent_positions: np.ndarray,
    task_positions: np.ndarray,
    task_values: np.ndarray,
    distance_weight: float = 1.0,
) -> np.ndarray:
    """由位置信息和任务价值构建效用矩阵。

    公式：utility(i, j) = task_value(j) - distance_weight * distance(agent_i, task_j)
    """

    agent_positions = np.asarray(agent_positions, dtype=np.float64)
    task_positions = np.asarray(task_positions, dtype=np.float64)
    task_values = np.asarray(task_values, dtype=np.float64)

    if agent_positions.ndim != 2 or task_positions.ndim != 2:
        raise ValueError("agent_positions and task_positions must be 2D arrays")

    if agent_positions.shape[1] != task_positions.shape[1]:
        raise ValueError("agent_positions and task_positions must share dimension")

    if task_values.ndim != 1 or task_values.size != task_positions.shape[0]:
        raise ValueError("task_values must be 1D with same length as task_positions")

    # 广播后得到两两差分张量：[n_agents, n_tasks, dim]。
    diff = agent_positions[:, None, :] - task_positions[None, :, :]
    distances = np.linalg.norm(diff, axis=2)
    utility = task_values[None, :] - distance_weight * distances
    return utility


def auction_assign(drones: List[Any], tasks: List[dict]) -> dict:
    """AirCombat 兼容的占位接口。

    保留原始轮转分配行为，确保旧调用方不需要改动。
    """

    # 保持该接口稳定，避免影响主循环中的既有调用。
    assignment = {}
    for index, drone in enumerate(drones):
        assignment[drone.name] = tasks[index % len(tasks)] if tasks else None
    return assignment


def greedy_assignment(
    defenders: List[Any],
    attackers: List[Any],
    config: ImprovedAuctionConfig | None = None,
) -> Dict[str, Optional[Any]]:
    """基于改进拍卖算法的 AirCombat 兼容分配接口。

    返回：{防守方名称: 进攻方对象或 None}
    """

    # 完全遵守旧接口契约：键是防守方 name，值是目标对象或 None。
    if not defenders:
        return {}

    if not attackers:
        return {d.name: None for d in defenders}

    # 直接基于 AirCombat 的 distance 构建效用矩阵（负距离越大越优）。
    utility = _build_distance_only_utility(defenders, attackers)

    solver = ImprovedAuctionSolver(config or ImprovedAuctionConfig())
    n_def, n_att = utility.shape

    assignment: Dict[str, Optional[Any]] = {d.name: None for d in defenders}

    if n_def >= n_att:
        # 拍卖求解结果含义：task_to_agent[进攻方索引] = 防守方索引。
        result = solver.solve(utility)
        for attacker_idx, defender_idx in enumerate(result.task_to_agent.tolist()):
            defender = defenders[int(defender_idx)]
            assignment[defender.name] = attackers[attacker_idx]
        return assignment

    # 当防守方少于进攻方时，对转置矩阵求解，保证每个防守方仍只匹配一个目标，
    # 同时保持一对一约束不被破坏。
    transposed_result = solver.solve(utility.T)
    for defender_idx, attacker_idx in enumerate(transposed_result.task_to_agent.tolist()):
        defender = defenders[defender_idx]
        assignment[defender.name] = attackers[int(attacker_idx)]

    return assignment


def _build_distance_only_utility(defenders: List[Any], attackers: List[Any]) -> np.ndarray:
    """构建“仅距离项”的效用矩阵（值越大越好）。

    由于目标是最小化距离，这里取负距离作为效用：utility = -distance。
    """

    if _aircombat_distance is None:
        raise ImportError(
            "未找到 utils.geometry.distance。请将本文件放入 AirCombat 仓库后再运行。"
        )

    n_def = len(defenders)
    n_att = len(attackers)
    utility = np.zeros((n_def, n_att), dtype=np.float64)

    for i, d in enumerate(defenders):
        for j, a in enumerate(attackers):
            utility[i, j] = -float(_aircombat_distance(d.position, a.position))
    return utility
