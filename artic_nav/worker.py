"""Spawned child: receives ONLY whitelisted calibration and sensor packets."""
import multiprocessing as mp
import traceback
import sys
import math
from .config import public_config


def worker_main(connection, cfg):
    from .navigation import Navigation
    navigator = Navigation(cfg)
    assert "mujoco" not in sys.modules and "artic_nav.physics" not in sys.modules
    try:
        while True:
            request = connection.recv()
            if request.get("action") == "close":
                break
            try:
                validate_request(request)
                connection.send(navigator.update(request))
            except Exception:
                connection.send({"error": traceback.format_exc()})
    except (EOFError, BrokenPipeError):
        pass
    finally:
        connection.close()


def validate_request(request):
    if set(request)-{"sensors", "action", "goal"}:
        raise ValueError("Non-public navigation request fields")
    for p in request.get("sensors", []):
        if not math.isfinite(p["time"]):
            raise ValueError("Non-finite sensor timestamp")
        if p["kind"] == "imu":
            if set(p) != {"kind", "time", "front", "rear"}:
                raise ValueError("IMU packet contains non-public fields")
            for name in ("front", "rear"):
                if set(p[name]) != {"accel", "gyro"}:
                    raise ValueError("IMU must expose only acceleration and gyro")
                for values in p[name].values():
                    if len(values) != 3 or not all(math.isfinite(v) for v in values):
                        raise ValueError("Invalid IMU vector")
        elif p["kind"] == "camera":
            if set(p) != {"kind", "time", "detections"}:
                raise ValueError("Camera packet contains non-public fields")
            for d in p["detections"]:
                if set(d) != {"class", "box", "confidence"}:
                    raise ValueError("Detection contains non-public fields")
                if len(d["box"]) != 4 or not all(math.isfinite(v) for v in d["box"]):
                    raise ValueError("Invalid detection box")
        else:
            raise ValueError("Unsupported sensor kind")


class NavigationProcess:
    def __init__(self, cfg):
        ctx = mp.get_context("spawn")
        self.connection, child = ctx.Pipe()
        self.process = ctx.Process(target=worker_main, args=(child, public_config(cfg)), daemon=True)
        self.process.start()
        child.close()
        self.timeout = cfg["navigation"]["rpc_timeout_seconds"]

    def request(self, sensors, action=None, goal=None):
        # Images are permitted, but this mock-detector navigator needs only boxes.
        clean = [{k: v for k, v in packet.items() if k != "rgb"} for packet in sensors]
        message = {"sensors": clean}
        if action:
            message["action"] = action
        if goal is not None:
            message["goal"] = list(goal)
        self.connection.send(message)
        if not self.connection.poll(self.timeout):
            raise TimeoutError("导航计算未响应；仿真时间已暂停，不启动替代任务")
        result = self.connection.recv()
        if "error" in result:
            raise RuntimeError(result["error"])
        return result

    def close(self):
        if self.process.is_alive():
            try:
                self.connection.send({"action": "close"})
            except (BrokenPipeError, EOFError):
                pass
            self.process.join(timeout=2)
            if self.process.is_alive():
                self.process.terminate()
                self.process.join(timeout=2)
        self.connection.close()
