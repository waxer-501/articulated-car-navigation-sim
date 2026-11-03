import argparse
import json
from pathlib import Path
import numpy as np
from .config import load_config


def run_episode(cfg, goal, curve=False, directory=None, video=False, blackout=None, quiet=False):
    from .session import Session
    session = Session(cfg, directory)
    if blackout:
        session.sensors.blackout = blackout
    if video:
        session.record(True)
    complete_at = None
    try:
        count = 0
        reason = "episode_deadline"
        while session.world.time < cfg["navigation"]["episode_seconds"]:
            output = session.tick(("curve" if curve else "goal") if count == 0 else None,
                                  goal if count == 0 and not curve else None, render=video)
            count += 1
            if not quiet and count % 50 == 0:
                print(json.dumps({"t": round(session.world.time, 1), "status": output["status"],
                                  "estimate": output["estimate"]}, ensure_ascii=False), flush=True)
            if output["mode"] == "failed":
                reason = "planning_failed"
                break
            if output["mode"] == "complete":
                if complete_at is None:
                    complete_at = session.world.time
                if session.world.time-complete_at >= 2:
                    reason = "navigator_complete"
                    break
        report = session.export(reason)
        if not quiet:
            print(json.dumps({"directory": str(session.directory), **report}, ensure_ascii=False, indent=2), flush=True)
        return report
    finally:
        session.close()


def main():
    parser = argparse.ArgumentParser(description="全新铰接小车仿真：无任何实车接口")
    parser.add_argument("command", choices=["gui", "run", "batch", "replay"], nargs="?", default="gui")
    parser.add_argument("--config")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--profile", choices=["ideal", "nominal", "stress"], default="nominal")
    parser.add_argument("--goal", nargs=3, type=float, default=[.45, .15, 45], metavar=("X", "Y", "FRONT_YAW_DEG"))
    parser.add_argument("--initial", nargs=4, type=float, metavar=("X", "Y", "FRONT_YAW_DEG", "ALPHA_DEG"))
    parser.add_argument("--curve", action="store_true")
    parser.add_argument("--seconds", type=float)
    parser.add_argument("--output")
    parser.add_argument("--video", action="store_true")
    parser.add_argument("--blackout", nargs=2, type=float)
    parser.add_argument("--jobs", type=int, default=2)
    parser.add_argument("--suite", choices=["acceptance", "stress", "sensitivity"], default="acceptance")
    parser.add_argument("--requests", help="sensor_requests.jsonl.gz for deterministic replay")
    args = parser.parse_args()
    cfg = load_config(args.config, args.seed, args.profile)
    if args.seconds:
        cfg["navigation"]["episode_seconds"] = args.seconds
    if args.initial:
        cfg["physics"]["initial_pose"] = [*args.initial[:2], *np.deg2rad(args.initial[2:]).tolist()]
    if args.command == "gui":
        from .gui import launch
        launch(cfg)
    elif args.command == "run":
        goal = [*args.goal[:2], float(np.deg2rad(args.goal[2]))]
        run_episode(cfg, goal, args.curve, args.output, args.video, args.blackout)
    elif args.command == "batch":
        from .benchmark import batch
        batch(cfg, args.suite, args.jobs, args.output)
    elif args.command == "replay":
        from .benchmark import replay
        if not args.requests:
            parser.error("replay requires --requests")
        print(json.dumps(replay(cfg, Path(args.requests)), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
