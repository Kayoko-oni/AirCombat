from __future__ import annotations

import time
from dataclasses import dataclass
from importlib import import_module
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

_MAX_BATTERY_REF = 120.0
_MAX_HEALTH_REF = 150.0
_MAX_SPEED_REF = 40.0
_MAX_ATTACK_POWER_REF = 30.0

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
    communication_range: float = 280.0
    max_operational_distance: float = 1200.0
    response_distance_weight: float = 0.55
    response_energy_weight: float = 0.30
    response_health_weight: float = 0.15
    response_threshold: float = 0.30
    attention_temperature: float = 0.70
    distance_weight: float = 1.0
    urgency_weight: float = 35.0
    load_balance_weight: float = 18.0
    feasibility_weight: float = 16.0
    attention_weight: float = 22.0
    non_responder_penalty: float = 55.0
    sticky_assignment_bonus: float = 10.0
    reassignment_penalty: float = 6.0


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

    cfg = config or ImprovedAuctionConfig()

    # 将论文中的“提议-响应-选择”思想落地为效用增强，而非仅做纯距离匹配。
    utility = _build_cma_enhanced_utility(defenders, attackers, cfg)

    solver = ImprovedAuctionSolver(cfg)
    n_def, n_att = utility.shape

    assignment: Dict[str, Optional[Any]] = {d.name: None for d in defenders}

    if n_def >= n_att:
        # 拍卖求解结果含义：task_to_agent[进攻方索引] = 防守方索引。
        result = solver.solve(utility)
        for attacker_idx, defender_idx in enumerate(result.task_to_agent.tolist()):
            defender = defenders[int(defender_idx)]
            assignment[defender.name] = attackers[attacker_idx]
        _remember_assignment(assignment)
        return assignment

    # 当防守方少于进攻方时，对转置矩阵求解，保证每个防守方仍只匹配一个目标，
    # 同时保持一对一约束不被破坏。
    transposed_result = solver.solve(utility.T)
    for defender_idx, attacker_idx in enumerate(transposed_result.task_to_agent.tolist()):
        defender = defenders[defender_idx]
        assignment[defender.name] = attackers[int(attacker_idx)]

    _remember_assignment(assignment)
    return assignment


_LAST_TARGET_BY_DEFENDER: Dict[str, str] = {}


def _remember_assignment(assignment: Dict[str, Optional[Any]]) -> None:
    """缓存上一帧分配结果，用于抑制频繁切换目标。"""
    global _LAST_TARGET_BY_DEFENDER
    next_mapping: Dict[str, str] = {}
    for defender_name, attacker in assignment.items():
        if attacker is not None:
            next_mapping[defender_name] = str(getattr(attacker, "name", ""))
    _LAST_TARGET_BY_DEFENDER = next_mapping


def _build_cma_enhanced_utility(
    defenders: List[Any],
    attackers: List[Any],
    cfg: ImprovedAuctionConfig,
) -> np.ndarray:
    """构建融合“提议-响应-选择”机制的效用矩阵。"""
    if _aircombat_distance is None:
        raise ImportError(
            "未找到 utils.geometry.distance。请将本文件放入 AirCombat 仓库后再运行。"
        )

    n_def = len(defenders)
    n_att = len(attackers)
    utility = np.zeros((n_def, n_att), dtype=np.float64)

    defender_loads = np.asarray([_estimate_defender_load(d) for d in defenders], dtype=np.float64)
    task_urgencies = np.asarray(
        [_estimate_attacker_urgency(a, cfg.max_operational_distance) for a in attackers],
        dtype=np.float64,
    )

    for attacker_idx, attacker in enumerate(attackers):
        # Stage-1 提议：选择离该任务最近的防守方作为提议者。
        proposer_idx = _select_proposer(defenders, attacker)
        # Stage-2 响应：提议者仅与通信半径内成员协商。
        candidate_indices = _collect_candidates(defenders, proposer_idx, cfg.communication_range)
        feasible_scores = _response_feasibility_scores(defenders, attacker, cfg, candidate_indices)

        feasible_map = {
            idx
            : score
            for idx, score in zip(candidate_indices, feasible_scores)
            if score >= cfg.response_threshold
        }
        if not feasible_map:
            feasible_map = {proposer_idx: 0.0}
        responders = list(feasible_map)
        responder_set = set(responders)

        # Stage-3 选择：在可响应集合内进行注意力匹配打分。
        attention_map = _cma_attention_scores(defenders, attacker, responders, cfg)
        urgency = float(task_urgencies[attacker_idx])

        for defender_idx, defender in enumerate(defenders):
            distance = float(_aircombat_distance(defender.position, attacker.position))
            base = -cfg.distance_weight * distance
            urgency_term = cfg.urgency_weight * urgency
            load_term = cfg.load_balance_weight * (1.0 - float(defender_loads[defender_idx]))

            responder_term = 0.0
            if defender_idx in responder_set:
                feasibility = float(feasible_map.get(defender_idx, 0.0))
                attention = float(attention_map.get(defender_idx, 0.0))
                responder_term += cfg.feasibility_weight * feasibility
                responder_term += cfg.attention_weight * attention
            else:
                # 非响应节点直接降权，避免通信受限时被误选。
                responder_term -= cfg.non_responder_penalty

            defender_name = str(getattr(defender, "name", ""))
            attacker_name = str(getattr(attacker, "name", ""))
            previous = _LAST_TARGET_BY_DEFENDER.get(defender_name)
            if previous == attacker_name:
                # 维持原锁定目标会获得粘性奖励，降低抖动重分配。
                switch_term = cfg.sticky_assignment_bonus
            elif previous is None:
                switch_term = 0.0
            else:
                # 有历史目标但本轮切换时施加惩罚。
                switch_term = -cfg.reassignment_penalty

            utility[defender_idx, attacker_idx] = (
                base + urgency_term + load_term + responder_term + switch_term
            )

    return utility


def _select_proposer(defenders: List[Any], attacker: Any) -> int:
    """按距离最近原则选择提议无人机。"""
    if not defenders:
        raise ValueError("defenders must be non-empty")
    best_idx = 0
    best_dist = float("inf")
    for idx, defender in enumerate(defenders):
        dist = float(_aircombat_distance(defender.position, attacker.position))
        if dist < best_dist:
            best_dist = dist
            best_idx = idx
    return best_idx


def _collect_candidates(
    defenders: List[Any],
    proposer_idx: int,
    communication_range: float,
) -> List[int]:
    """收集提议者通信范围内的候选防守方。"""
    proposer = defenders[proposer_idx]
    candidates = []
    for idx, defender in enumerate(defenders):
        d_comm = float(_aircombat_distance(proposer.position, defender.position))
        if d_comm <= communication_range:
            candidates.append(idx)
    if proposer_idx not in candidates:
        candidates.append(proposer_idx)
    return candidates


def _response_feasibility_scores(
    defenders: List[Any],
    attacker: Any,
    cfg: ImprovedAuctionConfig,
    candidate_indices: List[int],
) -> List[float]:
    """按距离/电量/血量计算响应可行性分数。"""
    scores: List[float] = []

    for idx in candidate_indices:
        defender = defenders[idx]
        distance_score = _pair_distance_score(defender, attacker, cfg.max_operational_distance)
        energy_score, health_score = _defender_resource_scores(defender)
        score = (
            cfg.response_distance_weight * distance_score
            + cfg.response_energy_weight * energy_score
            + cfg.response_health_weight * health_score
        )
        scores.append(float(np.clip(score, 0.0, 1.0)))

    return scores


def _cma_attention_scores(
    defenders: List[Any],
    attacker: Any,
    responder_indices: List[int],
    cfg: ImprovedAuctionConfig,
) -> Dict[int, float]:
    """用轻量注意力计算任务与候选防守方的匹配权重。"""
    if not responder_indices:
        return {}

    closeness, task_speed, task_attack, task_health = _attacker_scores(
        attacker, cfg.max_operational_distance
    )
    urgency = _estimate_attacker_urgency(attacker, cfg.max_operational_distance)

    q = np.asarray(
        [
            urgency,
            closeness,
            task_speed,
            task_attack,
            task_health,
        ],
        dtype=np.float64,
    )

    logits: List[float] = []
    for idx in responder_indices:
        defender = defenders[idx]
        # key 向量体现候选防守方对当前任务的执行能力。
        energy_score, health_score = _defender_resource_scores(defender)
        k = np.asarray(
            [
                _pair_distance_score(defender, attacker, cfg.max_operational_distance),
                _ratio(_speed(defender), _MAX_SPEED_REF),
                energy_score,
                health_score,
                1.0 - _estimate_defender_load(defender),
            ],
            dtype=np.float64,
        )
        score = float(np.dot(q, k) / np.sqrt(q.size))
        logits.append(score / max(cfg.attention_temperature, 1e-6))

    probs = _softmax(np.asarray(logits, dtype=np.float64))
    return {idx: float(p) for idx, p in zip(responder_indices, probs.tolist())}


def _estimate_attacker_urgency(attacker: Any, max_operational_distance: float = 1200.0) -> float:
    """估计任务紧急度代理值，输出范围 [0, 1]。"""
    closeness, speed_score, attack_score, health_score = _attacker_scores(
        attacker, max_operational_distance
    )
    urgency = 0.35 * closeness + 0.25 * speed_score + 0.25 * attack_score + 0.15 * health_score
    return float(np.clip(urgency, 0.0, 1.0))


def _estimate_defender_load(defender: Any) -> float:
    """估计防守方当前负载，值越大表示越繁忙。"""
    battery_ratio, health_ratio = _defender_resource_scores(defender)
    load = 0.7 * (1.0 - battery_ratio) + 0.3 * (1.0 - health_ratio)
    return float(np.clip(load, 0.0, 1.0))


def _attacker_scores(attacker: Any, max_operational_distance: float) -> tuple[float, float, float, float]:
    """提取任务端归一化特征：接近基地、速度、攻击力、血量。"""
    closeness = 1.0 - _ratio(_norm_to_base(attacker), max_operational_distance)
    speed_score = _ratio(_speed(attacker), _MAX_SPEED_REF)
    attack_score = _ratio(getattr(attacker, "attack_power", 0.0), _MAX_ATTACK_POWER_REF)
    health_score = _ratio(getattr(attacker, "health", 0.0), _MAX_HEALTH_REF)
    return closeness, speed_score, attack_score, health_score


def _defender_resource_scores(defender: Any) -> tuple[float, float]:
    """提取防守端归一化资源特征：电量、血量。"""
    energy_score = _ratio(getattr(defender, "battery", 0.0), _MAX_BATTERY_REF)
    health_score = _ratio(getattr(defender, "health", 0.0), _MAX_HEALTH_REF)
    return energy_score, health_score


def _pair_distance_score(defender: Any, attacker: Any, max_operational_distance: float) -> float:
    """将防守方与任务的距离映射为 [0, 1] 可行性分数。"""
    distance = float(_aircombat_distance(defender.position, attacker.position))
    return 1.0 - _ratio(distance, max_operational_distance)


def _speed(drone: Any) -> float:
    """计算无人机速度模长。"""
    velocity = np.asarray(getattr(drone, "velocity", [0.0, 0.0, 0.0]), dtype=np.float64)
    return float(np.linalg.norm(velocity))


def _norm_to_base(drone: Any) -> float:
    """计算无人机相对基地(原点)的距离。"""
    position = np.asarray(getattr(drone, "position", [0.0, 0.0, 0.0]), dtype=np.float64)
    return float(np.linalg.norm(position))


def _ratio(value: float, max_value: float) -> float:
    """将值归一化到 [0, 1]，并处理零分母。"""
    if max_value <= 0:
        return 0.0
    return float(np.clip(value / max_value, 0.0, 1.0))


def _softmax(x: np.ndarray) -> np.ndarray:
    """数值稳定版 softmax。"""
    x = x - np.max(x)
    ex = np.exp(x)
    denom = np.sum(ex)
    if denom <= 0:
        return np.full_like(ex, 1.0 / max(ex.size, 1))
    return ex / denom
