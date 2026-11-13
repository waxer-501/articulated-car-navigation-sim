"""Fixed, reproducible simulated acceptance, fault and parameter sweeps."""
import gzip
import hashlib
import json
from concurrent.futures import ProcessPoolExecutor, as_completed
from copy import deepcopy
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo
import numpy as np
from .config import ROOT, public_config


CASES = [
    ("straight", [-.55, -.45, 0, 0], [.10, -.45, 0]),
    ("reverse", [.35, -.45, 0, 0], [-.25, -.45, 0]),
    ("left_turn", [-.55, -.45, 0, 0], [.45, .15, 45]),
    ("right_turn", [-.55, .45, 0, 0], [.45, -.15, -45]),
    ("lane_change", [-.55, -.45, 0, 0], [.35, -.10, 0]),
    ("turnaround", [-.45, -.35, 0, 0], [.1, .35, 180]),
    ("near_boundary", [-.4, -.5, 0, 0], [.65, .60, 90]),
    ("angle_wrap", [.5, .2, 170, 0], [-.25, .25, -170]),
    ("bent_start", [-.45, -.4, 25, 20], [.4, .25, 30]),
    ("reverse_heading", [.5, .4, 90, 0], [.1, -.3, 90]),
]


def _job(cfg, goal, directory, curve=False, blackout=None):
    from .cli import run_episode
    return run_episode(cfg, goal, curve, str(directory), blackout=blackout, quiet=True)


def batch(cfg, suite="acceptance", jobs=2, output=None):
    stamp = datetime.now(ZoneInfo("Asia/Shanghai")).strftime("%Y%m%d_%H%M%S")
    root = Path(output) if output else ROOT/"runs"/(suite+"_"+stamp)
    root.mkdir(parents=True, exist_ok=False)
    tasks = []
    if suite == "acceptance":
        for name, initial, goal in CASES:
            for seed in range(5):
                local = deepcopy(cfg)
                local["seed"], local["profile"] = seed, "nominal"
                local["physics"]["initial_pose"] = [*initial[:2], *np.deg2rad(initial[2:]).tolist()]
                tasks.append((name+f"_seed{seed}", local, [*goal[:2], float(np.deg2rad(goal[2]))], False, None))
    elif suite == "stress":
        for profile in ("ideal", "nominal", "stress"):
            for seed in range(3):
                local = deepcopy(cfg)
                local["profile"], local["seed"] = profile, seed
                tasks.append((f"{profile}_{seed}", local, [.45, .15, np.pi/4], False, None))
        for seed in range(3):
            local = deepcopy(cfg)
            local["seed"] = seed
            tasks.append((f"blackout_{seed}", local, [.45, .15, np.pi/4], False, (3., 6.)))
            tasks.append((f"curve_{seed}", deepcopy(local), None, True, None))
        tasks.append(("unreachable", deepcopy(cfg), [.99, .99, 0.], False, None))
    elif suite == "steering":
        # Only hidden plant parameters vary; public prediction/controller priors
        # stay identical. Include both steering directions and a reversed start.
        plants = {
            "slow_left": {"steer_left_rate_deg": 18., "steer_right_rate_deg": 48.,
                          "steer_exponent": 1.6, "steer_deadzone": .12},
            "slow_right_delay": {"steer_left_rate_deg": 48., "steer_right_rate_deg": 22.,
                                 "actuator_delay": .15},
            "weak_both": {"steer_left_rate_deg": 18., "steer_right_rate_deg": 20.,
                          "steer_torque": .24},
            "fast_load_drift": {"steer_left_rate_deg": 48., "steer_right_rate_deg": 55.,
                                "steer_angle_loss": .35, "steer_drift_fraction": .20},
        }
        for name, plant in plants.items():
            for seed in range(2):
                for case in ("left_turn", "right_turn"):
                    _, initial, goal = next(row for row in CASES if row[0] == case)
                    local = deepcopy(cfg)
                    local["seed"] = seed
                    local["physics"].update(plant)
                    local["physics"]["initial_pose"] = [*initial[:2], *np.deg2rad(initial[2:]).tolist()]
                    tasks.append((f"{name}_{case}_{seed}", local, [*goal[:2], float(np.deg2rad(goal[2]))], False, None))
        for seed in range(3):
            local = deepcopy(cfg)
            local["seed"] = seed
            tasks.append((f"curve_{seed}", local, None, True, None))
        tasks.append(("blackout", deepcopy(cfg), [.45, .15, np.pi/4], False, (3., 6.)))
    elif suite == "sensitivity":
        for parameter, values in (("mass", [.7, 1.3]), ("friction", [.45, 1.1]),
                                  ("drive_torque", [.028, .042]), ("actuator_delay", [.03, .15])):
            for value in values:
                local = deepcopy(cfg)
                local["physics"][parameter] = value
                tasks.append((f"{parameter}_{value}", local, [.45, .15, np.pi/4], False, None))
    else:
        raise ValueError("Unknown validation suite")
    reports = {}
    with ProcessPoolExecutor(max_workers=jobs) as pool:
        futures = {pool.submit(_job, local, goal, root/name, curve, blackout): name
                   for name, local, goal, curve, blackout in tasks}
        for future in as_completed(futures):
            name = futures[future]
            try:
                report = future.result()
            except Exception as exc:
                report = {"success": False, "error": repr(exc), "boundary_ok": False}
            reports[name] = report
            print(json.dumps({"case": name, "success": report["success"], "position_error_m": report.get("position_error_m"),
                              "heading_error_deg": report.get("front_heading_error_deg"),
                              "completed": len(reports), "total": len(tasks)}, ensure_ascii=False), flush=True)
            (root/"partial_results.json").write_text(json.dumps(reports, ensure_ascii=False, indent=2), encoding="utf-8")
    successes = sum(r["success"] for r in reports.values())
    summary = {"suite": suite, "runs": len(tasks), "successes": successes, "success_rate": successes/len(tasks),
               "all_within_boundary": all(r.get("boundary_ok", False) for r in reports.values()),
               "all_within_articulation_limit": all(r.get("articulation_ok", False) for r in reports.values()),
               "acceptance_pass": suite == "acceptance" and successes/len(tasks) >= .9
                                  and all(r.get("boundary_ok", False) and r.get("articulation_ok", False) for r in reports.values()),
               "cases": reports}
    (root/"summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({k: v for k, v in summary.items() if k != "cases"}, ensure_ascii=False, indent=2))
    return summary


def replay(cfg, requests):
    """No truth path is accepted. Replays exactly the allowed observation stream."""
    from .worker import NavigationProcess
    nav = NavigationProcess(cfg)
    digest = hashlib.sha256()
    count = 0
    try:
        with gzip.open(requests, "rt", encoding="utf-8") as stream:
            for line in stream:
                message = json.loads(line)
                result = nav.request(message["sensors"], message.get("action"), message.get("goal"))
                encoded = json.dumps(result, sort_keys=True, allow_nan=False).encode()
                digest.update(encoded)
                count += 1
    finally:
        nav.close()
    return {"messages": count, "navigation_sha256": digest.hexdigest(), "source": str(requests), "truth_input": False}
