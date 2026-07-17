"""server 侧状态库。

关键不变量：robtarget 数组、关节路径这类大对象**留在这里**，MCP 工具只返回
摘要 + ID。这样既不撑爆对话上下文，也守住 hermes「对话前缀缓存不可变」的要求
——几十轮 ReAct 迭代不会把历史坐标灌进 system prompt。

当前为进程内实现；需要跨会话持久化时，在此换成 SQLite/JSON 落盘即可，接口不变。
"""

from __future__ import annotations

import threading
import uuid
from typing import TypeVar

from .types import GrindPlan, GrindSpec, GrindStep, SimResult, TargetSet, Workpiece

T = TypeVar("T")


def new_id(prefix: str) -> str:
    return f"{prefix}_{uuid.uuid4().hex[:12]}"


class Ledger:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._specs: dict[str, GrindSpec] = {}
        self._steps: dict[str, GrindStep] = {}
        self._targets: dict[str, TargetSet] = {}
        self._sims: dict[str, SimResult] = {}
        self._workpieces: dict[str, Workpiece] = {}
        self._plans: dict[str, GrindPlan] = {}

    # --- specs ---
    def put_spec(self, spec: GrindSpec) -> None:
        with self._lock:
            self._specs[spec.spec_id] = spec

    def get_spec(self, spec_id: str) -> GrindSpec:
        return self._require(self._specs, spec_id, "spec")

    # --- steps ---
    def put_step(self, step: GrindStep) -> None:
        with self._lock:
            self._steps[step.step_id] = step

    def get_step(self, step_id: str) -> GrindStep:
        return self._require(self._steps, step_id, "step")

    def steps_for_spec(self, spec_id: str) -> list[GrindStep]:
        with self._lock:
            steps = [s for s in self._steps.values() if s.spec_id == spec_id]
        return sorted(steps, key=lambda s: s.order)

    # --- target sets ---
    def put_targets(self, ts: TargetSet) -> None:
        with self._lock:
            self._targets[ts.targets_id] = ts

    def get_targets(self, targets_id: str) -> TargetSet:
        return self._require(self._targets, targets_id, "targets")

    # --- sim results ---
    def put_sim(self, sim: SimResult) -> None:
        with self._lock:
            self._sims[sim.sim_id] = sim

    def get_sim(self, sim_id: str) -> SimResult:
        return self._require(self._sims, sim_id, "sim")

    # --- workpieces（点云留 server 侧，用 ID 引用） ---
    def put_workpiece(self, wp: Workpiece) -> None:
        with self._lock:
            self._workpieces[wp.workpiece_id] = wp

    def get_workpiece(self, workpiece_id: str) -> Workpiece:
        return self._require(self._workpieces, workpiece_id, "workpiece")

    # --- plans ---
    def put_plan(self, plan: GrindPlan) -> None:
        with self._lock:
            self._plans[plan.plan_id] = plan

    def get_plan(self, plan_id: str) -> GrindPlan:
        return self._require(self._plans, plan_id, "plan")

    # --- 内部 ---
    def _require(self, store: dict[str, T], key: str, kind: str) -> T:
        with self._lock:
            if key not in store:
                raise KeyError(f"未找到 {kind} '{key}'（可能已过期或 ID 有误）")
            return store[key]
