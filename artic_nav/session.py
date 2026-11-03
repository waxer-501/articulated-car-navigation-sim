import gzip
import json
import time
import hashlib
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo
import numpy as np
import yaml
from .config import ROOT
from .physics import World
from .sensors import Sensors
from .worker import NavigationProcess
from .evaluation import Evaluator


class Session:
    def __init__(self, cfg, output_dir=None):
        self.cfg = cfg
        stamp = datetime.now(ZoneInfo("Asia/Shanghai")).strftime("%Y%m%d_%H%M%S_%f")
        self.directory = Path(output_dir) if output_dir else ROOT / "runs" / stamp
        self.directory.mkdir(parents=True, exist_ok=False)
        (self.directory/"config.yaml").write_text(yaml.safe_dump(cfg, allow_unicode=True), encoding="utf-8")
        sources = sorted((ROOT/"artic_nav").glob("*.py"))+[ROOT/"requirements.lock"]
        manifest = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources if p.exists()}
        (self.directory/"source_manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
        self.world = World(cfg)
        (self.directory/"model.xml").write_text(__import__("artic_nav.physics", fromlist=["make_xml"]).make_xml(cfg), encoding="utf-8")
        self.sensors = Sensors(self.world, cfg)
        self.navigator = NavigationProcess(cfg)
        self.evaluator = Evaluator(cfg)
        self.inputs = gzip.open(self.directory/"sensor_requests.jsonl.gz", "wt", encoding="utf-8")
        self.controls = open(self.directory/"navigation.jsonl", "w", encoding="utf-8")
        self.truth_file = open(self.directory/"truth_evaluation_only.jsonl", "w", encoding="utf-8")
        self.output = {"mode": "idle", "status": "初始化", "estimate": None}
        self.camera = None
        self.recording = None
        self.curve = False
        self.closed = False
        self.last_path = None
        self.wall_start = time.monotonic()

    def tick(self, action=None, goal=None, render=False):
        packets = []
        steps = round(1/self.cfg["navigation"]["control_hz"]/self.cfg["physics"]["dt"])
        for i in range(steps):
            self.world.step()
            sampled = self.sensors.sample(rgb=render)
            for packet in sampled:
                if packet["kind"] == "camera":
                    self.camera = packet
                    if self.recording is not None and "rgb" in packet:
                        self.recording.append_data(packet["rgb"])
            packets.extend(sampled)
            if i % 4 == 0:
                self.evaluator.sample(self.world.truth())
        message = {"sensors": [{k: v for k, v in p.items() if k != "rgb"} for p in packets]}
        if action:
            message["action"] = action
        if goal is not None:
            message["goal"] = list(goal)
        if action == "curve":
            self.curve = True
        elif action == "goal":
            self.curve = False
        if action in ("goal", "curve"):
            self.evaluator = Evaluator(self.cfg)
        self.inputs.write(json.dumps(message, allow_nan=False)+"\n")
        self.output = self.navigator.request(packets, action, goal)
        self.world.command(self.output["command"])
        truth = self.world.truth()
        self.evaluator.sample(truth, self.output)
        self.truth_file.write(json.dumps(truth)+"\n")
        record = {k: v for k, v in self.output.items() if k != "path"}
        self.controls.write(json.dumps(record, allow_nan=False)+"\n")
        if self.output.get("path") is not None:
            if self.output["path"] != self.last_path:
                (self.directory/"planned_path.json").write_text(json.dumps(self.output["path"]), encoding="utf-8")
                with open(self.directory/"path_updates.jsonl", "a", encoding="utf-8") as stream:
                    stream.write(json.dumps({"time": self.world.time, "path": self.output["path"]})+"\n")
                self.last_path = self.output["path"]
        return self.output

    def stop(self):
        self.world.emergency_stop()
        return self.tick("stop")

    def record(self, enabled):
        if enabled and self.recording is None:
            import imageio.v2 as imageio
            self.recording = imageio.get_writer(self.directory/"overhead.mp4", fps=self.cfg["camera"]["hz"], codec="libx264")
        elif not enabled and self.recording is not None:
            self.recording.close()
            self.recording = None

    def export(self, reason="user_export"):
        report = self.evaluator.report(self.output, reason, self.curve)
        report["wall_time_s"] = time.monotonic()-self.wall_start
        (self.directory/"report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
        for stream in (self.inputs, self.controls, self.truth_file):
            stream.flush()
        return report

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.record(False)
        self.navigator.close()
        self.world.close()
        for stream in (self.inputs, self.controls, self.truth_file):
            stream.close()
