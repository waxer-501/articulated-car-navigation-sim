"""Desktop observer UI. Truth is used ONLY in the explicitly labelled evaluation panel."""
import json
import ctypes
import queue
import sys
import time
from pathlib import Path
import numpy as np
from PySide6.QtCore import Qt, QThread, Signal, QPointF, QRectF
from PySide6.QtGui import QColor, QFont, QImage, QPainter, QPen, QPixmap
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QLabel, QPushButton,
                              QHBoxLayout, QVBoxLayout, QComboBox, QFileDialog, QMessageBox,
                              QFrame, QGridLayout)
from .geometry import ground_from_pixel, project, footprint
from .config import ROOT

# Ubuntu 22.04 may omit this optional Qt xcb dependency. Load the bundled
# Ubuntu package locally; no system library installation or sudo is needed.
_cursor_library = ROOT / ".runtime/usr/lib/x86_64-linux-gnu/libxcb-cursor.so.0"
if _cursor_library.exists():
    ctypes.CDLL(str(_cursor_library), mode=ctypes.RTLD_GLOBAL)


class Engine(QThread):
    updated = Signal(object)
    failed = Signal(str)
    exported = Signal(str)

    def __init__(self, cfg):
        super().__init__()
        self.cfg = cfg
        self.commands = queue.Queue()
        self.stopping = False
        self.paused = False

    def run(self):
        from .session import Session
        session = None
        try:
            session = Session(self.cfg)
            while not self.stopping:
                started = time.monotonic()
                action, goal = None, None
                while not self.commands.empty():
                    command, value = self.commands.get_nowait()
                    if command == "pause":
                        self.paused = value
                    elif command == "reset":
                        session.export("reset")
                        session.close()
                        session = Session(self.cfg)
                        self.paused = False
                    elif command == "record":
                        session.record(value)
                    elif command == "export":
                        session.export()
                        self.exported.emit(str(session.directory))
                    elif command == "profile":
                        # A new experiment is required when noise profile changes.
                        self.cfg["profile"] = value
                        session.export("profile_changed")
                        session.close()
                        session = Session(self.cfg)
                    else:
                        action, goal = command, value
                        if command == "stop":
                            session.world.emergency_stop()
                            self.paused = False
                if self.paused:
                    self.msleep(30)
                    continue
                output = session.tick(action, goal, render=True)
                frame = session.camera
                self.updated.emit({"output": output, "camera": frame,
                                   "observer": session.world.render("observer"),
                                   "truth": session.world.truth(), "directory": str(session.directory)})
                elapsed = time.monotonic()-started
                if elapsed < .1:
                    self.msleep(round((.1-elapsed)*1000))
            session.export("window_closed")
        except Exception:
            import traceback
            self.failed.emit(traceback.format_exc())
        finally:
            if session is not None:
                session.close()


class CameraPanel(QWidget):
    targetSelected = Signal(object)

    def __init__(self, cfg):
        super().__init__()
        self.cfg = cfg
        self.frame = None
        self.output = {}
        self.image = None
        self.press = None
        self.target = None
        self.trail = []
        self.setMinimumSize(420, 420)
        self.setMouseTracking(True)

    def image_rect(self):
        side = min(self.width(), self.height())-16
        return QRectF((self.width()-side)/2, (self.height()-side)/2, side, side)

    def to_pixel(self, pos):
        rect = self.image_rect()
        if not rect.contains(pos):
            return None
        return np.array([(pos.x()-rect.x())/rect.width()*self.cfg["camera"]["width"],
                         (pos.y()-rect.y())/rect.height()*self.cfg["camera"]["height"]])

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.press = self.to_pixel(event.position())

    def mouseReleaseEvent(self, event):
        end = self.to_pixel(event.position())
        if self.press is not None and end is not None:
            start_xy = ground_from_pixel(self.press, self.cfg["camera"])
            end_xy = ground_from_pixel(end, self.cfg["camera"])
            if np.linalg.norm(end_xy-start_xy) > .025:
                yaw = np.arctan2(*(end_xy-start_xy)[::-1])
            else:
                yaw = self.output.get("estimate", [0, 0, 0, 0])
                yaw = yaw[2] if yaw is not None else 0.
            self.target = [*start_xy, float(yaw)]
            self.targetSelected.emit(self.target)
            self.update()
        self.press = None

    def update_data(self, frame, output):
        self.frame, self.output = frame, output
        if frame is not None and "rgb" in frame:
            arr = np.ascontiguousarray(frame["rgb"])
            self.image = QImage(arr.data, arr.shape[1], arr.shape[0], arr.strides[0], QImage.Format_RGB888).copy()
        if output.get("estimate") is not None:
            self.trail.append(output["estimate"][:2])
            self.trail = self.trail[-3000:]
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        rect = self.image_rect()
        painter.fillRect(self.rect(), QColor("#101923"))
        if self.image is None:
            painter.setPen(QColor("#91a8bb"))
            painter.drawText(rect, Qt.AlignCenter, "正在初始化相机与独立导航进程…")
            return
        painter.drawImage(rect, self.image)
        sx = rect.width()/self.cfg["camera"]["width"]
        sy = rect.height()/self.cfg["camera"]["height"]
        def point(uv):
            return QPointF(rect.x()+uv[0]*sx, rect.y()+uv[1]*sy)
        if self.output.get("path"):
            path = np.asarray(self.output["path"])
            uv = project(np.c_[path[:, :2], np.zeros(len(path))], self.cfg["camera"])
            painter.setPen(QPen(QColor("#f4d06b"), 2, Qt.DashLine))
            for a, b in zip(uv[:-1], uv[1:]):
                painter.drawLine(point(a), point(b))
        if len(self.trail) > 1:
            uv = project(np.c_[self.trail, np.zeros(len(self.trail))], self.cfg["camera"])
            painter.setPen(QPen(QColor("#62e7ee"), 2))
            for a, b in zip(uv[:-1], uv[1:]):
                painter.drawLine(point(a), point(b))
        if self.frame:
            for detection in self.frame["detections"]:
                x1, y1, x2, y2 = detection["box"]
                color = "#54e3d4" if detection["class"] == "front" else "#ffad68"
                painter.setPen(QPen(QColor(color), 2))
                painter.drawRect(QRectF(point([x1, y1]), point([x2, y2])))
                label = "前车体" if detection["class"] == "front" else "后车体"
                label_y = y1-5 if detection["class"] == "front" else y2+18
                painter.drawText(point([x1, label_y]), f"{label}  {detection['confidence']:.2f}")
        if self.target:
            x, y, yaw = self.target
            uv = project(np.array([[x, y, 0], [x+.13*np.cos(yaw), y+.13*np.sin(yaw), 0]]), self.cfg["camera"])
            painter.setPen(QPen(QColor("#ffffff"), 3))
            painter.drawEllipse(point(uv[0]), 7, 7)
            painter.drawLine(point(uv[0]), point(uv[1]))


class EvaluationPanel(QWidget):
    def __init__(self):
        super().__init__()
        self.truth = []
        self.estimated = []
        self.setMinimumSize(300, 200)

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor("#101923"))
        side = min(self.width()-28, self.height()-28)
        ox, oy = self.width()/2, self.height()/2
        p.setPen(QPen(QColor("#425568"), 1))
        p.drawRect(QRectF(ox-side/2, oy-side/2, side, side))
        for points, color in ((self.truth, "#ffbd73"), (self.estimated, "#62e7ee")):
            p.setPen(QPen(QColor(color), 2))
            for a, b in zip(points[:-1], points[1:]):
                p.drawLine(QPointF(ox+a[0]*side/2, oy-a[1]*side/2), QPointF(ox+b[0]*side/2, oy-b[1]*side/2))


class Window(QMainWindow):
    def __init__(self, cfg, start_engine=True):
        super().__init__()
        self.cfg = cfg
        self.target = None
        self.last_data = None
        self.setWindowTitle("铰接小车 · 传感器受限导航实验室")
        self.resize(1420, 960)
        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        layout.setContentsMargins(24, 20, 24, 16)
        title = QLabel("铰接小车  /  导航实验室")
        title.setObjectName("title")
        layout.addWidget(title)
        subtitle = QLabel("2 m × 2 m  ·  四轮物理接触  ·  顶置模拟视觉 + 双 IMU  ·  独立导航进程")
        subtitle.setObjectName("subtitle")
        layout.addWidget(subtitle)
        toolbar = QHBoxLayout()
        self.start_button = QPushButton("开始导航")
        self.start_button.clicked.connect(self.start_goal)
        self.start_button.setObjectName("primary")
        toolbar.addWidget(self.start_button)
        curve = QPushButton("S 曲线演示")
        curve.clicked.connect(lambda: self.send("curve"))
        toolbar.addWidget(curve)
        stop = QPushButton("停车")
        stop.setObjectName("stop")
        stop.clicked.connect(lambda: self.send("stop"))
        toolbar.addWidget(stop)
        pause = QPushButton("暂停仿真")
        self.pause_button = pause
        pause.setCheckable(True)
        pause.toggled.connect(lambda checked: (pause.setText("继续仿真" if checked else "暂停仿真"), self.send("pause", checked)))
        toolbar.addWidget(pause)
        reset = QPushButton("重置")
        reset.clicked.connect(self.reset)
        toolbar.addWidget(reset)
        self.record_button = QPushButton("录像")
        self.record_button.setCheckable(True)
        self.record_button.toggled.connect(lambda checked: (self.record_button.setText("结束录像" if checked else "录像"), self.send("record", checked)))
        toolbar.addWidget(self.record_button)
        export = QPushButton("导出结果")
        export.clicked.connect(lambda: self.send("export"))
        toolbar.addWidget(export)
        toolbar.addStretch()
        self.profile = QComboBox()
        self.profile.addItem("默认检测误差", "nominal")
        self.profile.addItem("理想检测", "ideal")
        self.profile.addItem("压力测试", "stress")
        self.profile.setCurrentIndex(self.profile.findData(cfg["profile"]))
        self.profile.currentIndexChanged.connect(lambda: (self.clear_trails(), self.record_button.setChecked(False), self.send("profile", self.profile.currentData())))
        toolbar.addWidget(self.profile)
        layout.addLayout(toolbar)
        body = QHBoxLayout()
        left = QVBoxLayout()
        left.addWidget(QLabel("导航观测  ·  在画面中按住并拖动：起点为目标位置，方向为前车朝向"))
        self.camera = CameraPanel(cfg)
        self.camera.targetSelected.connect(self.selected)
        left.addWidget(self.camera, 1)
        self.target_label = QLabel("尚未选择目标  ·  青色：估计轨迹    黄色虚线：规划路径")
        left.addWidget(self.target_label)
        body.addLayout(left, 3)
        right = QVBoxLayout()
        right.addWidget(QLabel("三维观察视图  ·  仅供人查看，不输入导航"))
        self.observer = QLabel()
        self.observer.setMinimumSize(330, 300)
        self.observer.setAlignment(Qt.AlignCenter)
        right.addWidget(self.observer, 3)
        right.addWidget(QLabel("独立评估视图  ·  橙色真值 / 青色估计，真值不回流"))
        self.evaluation = EvaluationPanel()
        right.addWidget(self.evaluation, 2)
        body.addLayout(right, 2)
        layout.addLayout(body, 1)
        self.metrics = QLabel("仿真时间 —    估计位置 —    前车朝向 —    铰接角 —")
        self.metrics.setObjectName("metrics")
        layout.addWidget(self.metrics)
        self.steering_metrics = QLabel("转向反馈：等待双 IMU 观测")
        self.steering_metrics.setWordWrap(True)
        layout.addWidget(self.steering_metrics)
        self.status = QLabel("正在启动…")
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        notice = QLabel("模拟检测框由仿真真值投影并加入误差；不代表真实 YOLO 能力。执行器与车体质量为可配置假设。")
        notice.setObjectName("subtitle")
        layout.addWidget(notice)
        self.setStyleSheet("""
            QMainWindow, QWidget { background: #172330; color: #e7eff6; font-size: 13px; }
            QLabel#title { font-size: 25px; font-weight: 700; padding-bottom: 3px; }
            QLabel#subtitle { color: #9bb0c3; font-size: 12px; padding-bottom: 8px; }
            QLabel#metrics { background: #223344; padding: 12px; border-radius: 7px; font-size: 14px; }
            QPushButton { background: #2b4053; border: 1px solid #3d566c; padding: 10px 14px; border-radius: 6px; }
            QPushButton:hover { background: #3b566b; } QPushButton:checked { background: #426776; }
            QPushButton#primary { background: #147d80; border-color: #279d9b; font-weight: bold; }
            QPushButton#stop { color: #ffb8ac; border-color: #87554c; }
            QComboBox { padding: 9px; background: #263a4b; border: 1px solid #3d566c; border-radius: 6px; }
        """)
        self.engine = Engine(cfg)
        self.engine.updated.connect(self.update_data)
        self.engine.failed.connect(self.failure)
        self.engine.exported.connect(lambda path: self.status.setText("已导出完整记录与 report.json："+path))
        if start_engine:
            self.engine.start()

    def send(self, command, value=None):
        if command in ("goal", "curve", "stop", "reset", "profile") and self.pause_button.isChecked():
            self.pause_button.setChecked(False)
        self.engine.commands.put((command, value))

    def selected(self, goal):
        self.target = goal
        self.target_label.setText(f"目标铰接轴：({goal[0]:.3f}, {goal[1]:.3f}) m    前车朝向：{np.rad2deg(goal[2]):.1f}°")

    def start_goal(self):
        if self.target is None:
            self.status.setText("请先在顶置画面拖动选择目标位置和前车朝向。")
        else:
            self.send("goal", self.target)

    def clear_trails(self):
        self.camera.trail.clear()
        self.evaluation.truth.clear()
        self.evaluation.estimated.clear()

    def reset(self):
        self.clear_trails()
        self.record_button.setChecked(False)
        self.send("reset")

    def update_data(self, data):
        self.last_data = data
        output = data["output"]
        self.camera.update_data(data["camera"], output)
        arr = np.ascontiguousarray(data["observer"])
        image = QImage(arr.data, arr.shape[1], arr.shape[0], arr.strides[0], QImage.Format_RGB888).copy()
        self.observer.setPixmap(QPixmap.fromImage(image).scaled(self.observer.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))
        self.evaluation.truth.append(data["truth"]["state"][:2])
        estimate = output.get("estimate")
        if estimate:
            self.evaluation.estimated.append(estimate[:2])
            sigma = output.get("position_sigma", 0.)
            self.metrics.setText(f"仿真 {output['time']:.1f} s    估计位置 ({estimate[0]:.3f}, {estimate[1]:.3f}) m    "
                                 f"前车朝向 {np.rad2deg(estimate[2]):.1f}°    铰接角 {np.rad2deg(estimate[3]):.1f}°    位置 σ {sigma*1000:.0f} mm")
        self.evaluation.update()
        steering = output.get("steering_feedback", {})
        if steering.get("fresh") and steering.get("rate_estimate") is not None:
            updates = steering["gain_updates"]
            self.steering_metrics.setText(
                f"转向遥控量 {steering['remote_value']:.3f}（0 左 / 0.5 保持 / 1 右）    "
                f"铰接速率：目标 {np.rad2deg(steering['target_rate']):.1f}°/s / "
                f"双 IMU 估计 {np.rad2deg(steering['rate_estimate']):.1f}°/s    "
                f"左右响应学习次数 {updates[0]} / {updates[1]}")
        else:
            self.steering_metrics.setText("转向反馈：等待新鲜的双 IMU 观测")
        self.status.setText(output["status"])

    def failure(self, message):
        self.status.setText("仿真已停止："+message[-500:])
        QMessageBox.critical(self, "仿真错误", message)

    def closeEvent(self, event):
        self.engine.stopping = True
        if not self.engine.wait(3000):
            self.status.setText("正在结束导航计算并保存记录，请稍候…")
            event.ignore()
            return
        event.accept()


def launch(cfg):
    app = QApplication.instance() or QApplication(sys.argv)
    app.setFont(QFont("Noto Sans CJK SC", 10))
    window = Window(cfg)
    window.show()
    sys.exit(app.exec())
