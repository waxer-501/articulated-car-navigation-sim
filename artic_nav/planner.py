"""Four-state Hybrid A*: pivot x/y, front yaw and articulation."""
import heapq
import itertools
import math
import numpy as np
from .geometry import advance, inside, wrap


class PlanningError(RuntimeError):
    pass


class HybridAStar:
    def __init__(self, cfg):
        self.g, self.n = cfg["geometry"], cfg["navigation"]
        self.expanded = 0

    def key(self, s):
        return (round(s[0]/self.n["planner_resolution"]), round(s[1]/self.n["planner_resolution"]),
                round(wrap(s[2])/np.deg2rad(self.n["planner_heading_deg"])),
                round(s[3]/np.deg2rad(self.n["planner_alpha_deg"])))

    def plan(self, start, goal, margin=None):
        margin = self.n["footprint_margin"] if margin is None else margin
        goal = np.asarray(goal)
        if not inside(start, self.g, margin):
            raise PlanningError("起点完整车身距离边界过近")
        if not any(inside([*goal, a], self.g, margin) for a in np.linspace(-.69, .69, 15)):
            raise PlanningError("目标位置和朝向无法容纳完整车身")
        def heuristic(s):
            distance = np.linalg.norm(s[:2]-goal[:2])
            return distance + .16*abs(wrap(s[2]-goal[2]))
        counter = itertools.count()
        heap = []
        nodes = [(np.array(start), -1, None, 0., 1)]
        best = {self.key(start): 0.}
        heapq.heappush(heap, (1.8*heuristic(np.array(start)), next(counter), 0))
        speed = min(.12, self.n["max_speed"])
        rate = np.deg2rad(self.n["max_steer_rate_deg"])
        self.expanded = 0
        found = None
        while heap and self.expanded < self.n["planner_max_nodes"]:
            _, _, idx = heapq.heappop(heap)
            state, parent, segment, cost, old_direction = nodes[idx]
            if cost > best.get(self.key(state), np.inf)+1e-8:
                continue
            self.expanded += 1
            if np.linalg.norm(state[:2]-goal[:2]) < .035 and abs(wrap(state[2]-goal[2])) < np.deg2rad(5):
                found = idx
                break
            # Shorter primitives in the final neighbourhood make exact stops feasible.
            duration = .6 if np.linalg.norm(state[:2]-goal[:2]) > .18 else .3
            dt = .1
            for direction in (1, -1):
                for steer in (-rate, 0., rate):
                    s = state.copy()
                    seg = []
                    valid = True
                    for _ in range(round(duration/dt)):
                        s = advance(s, direction*speed, steer, dt, self.g)
                        if not inside(s, self.g, margin):
                            valid = False
                            break
                        seg.append([*s, direction*speed, steer])
                    if not valid:
                        continue
                    key = self.key(s)
                    new_cost = cost+duration*speed*(1 if direction > 0 else 1.15)+.012*abs(steer)*duration
                    if old_direction != direction:
                        new_cost += .08
                    if new_cost >= best.get(key, np.inf):
                        continue
                    best[key] = new_cost
                    nodes.append((s, idx, seg, new_cost, direction))
                    heapq.heappush(heap, (new_cost+1.8*heuristic(s), next(counter), len(nodes)-1))
        if found is None:
            raise PlanningError(f"未找到可行路径（已搜索 {self.expanded} 个状态）")
        segments = []
        while nodes[found][1] != -1:
            segments.append(nodes[found][2])
            found = nodes[found][1]
        rows = [[*start, 0., 0.]]
        for seg in reversed(segments):
            rows.extend(seg)
        return np.asarray(rows)


def curve_reference(start, cfg):
    """A reproducible smooth S curve, generated from the public prediction model."""
    state = np.array(start, float)
    rows = [[*state, 0., 0.]]
    dt, speed, duration = .1, .085, 12.
    for i in range(round(duration/dt)):
        rate = .17*np.cos(2*np.pi*i*dt/duration)
        state = advance(state, speed, rate, dt, cfg["geometry"])
        if not inside(state, cfg["geometry"], .05):
            raise PlanningError("此起点的演示曲线会越界；请重置到默认起点")
        rows.append([*state, speed, rate])
    return np.asarray(rows)
