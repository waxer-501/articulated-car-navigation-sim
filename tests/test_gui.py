"""Keep Qt's native teardown separate from the MuJoCo test interpreter."""
import os
import subprocess
import sys


def test_drag_target_and_controls():
    script = r'''
from PySide6.QtCore import Qt, QPoint
from PySide6.QtTest import QTest
from PySide6.QtGui import QPainter, QImage
from PySide6.QtCore import QPointF
from PySide6.QtWidgets import QApplication
from artic_nav.config import load_config
from artic_nav.gui import Window
app = QApplication([])
# Exercise the exact void-returning drawing overload that crashed after many
# repaints with PySide 6.12 / Python 3.10 on this host.
image = QImage(64, 64, QImage.Format_RGB32)
painter = QPainter(image)
for i in range(100000):
    painter.drawLine(QPointF(0., 0.), QPointF(50., 50.))
painter.end()
window = Window(load_config(), start_engine=False)
window.show()
app.processEvents()
center = window.camera.rect().center()
QTest.mousePress(window.camera, Qt.LeftButton, pos=center)
QTest.mouseRelease(window.camera, Qt.LeftButton, pos=center+QPoint(60, -30))
assert window.target is not None
assert 0.2 < window.target[2] < .8
window.start_goal()
action, value = window.engine.commands.get_nowait()
assert action == "goal" and value == window.target
window.pause_button.setChecked(True)
assert window.engine.commands.get_nowait() == ("pause", True)
window.start_goal()
assert not window.pause_button.isChecked()
assert window.engine.commands.get_nowait() == ("pause", False)
assert window.engine.commands.get_nowait()[0] == "goal"
window.send("stop")
assert window.engine.commands.get_nowait()[0] == "stop"
import numpy as np
data = {"camera": None, "observer": np.zeros((64, 64, 3), dtype=np.uint8),
        "truth": {"state": [0, 0, 0, 0]},
        "output": {"time": 2., "estimate": None, "status": "testing", "steering_feedback": {
            "fresh": True, "target_rate": np.deg2rad(10.), "rate_estimate": np.deg2rad(7.),
            "remote_value": .2, "gain_updates": [3, 4]}}}
window.update_data(data)
assert "目标 10.0" in window.steering_metrics.text()
assert "估计 7.0" in window.steering_metrics.text()
assert "0.200" in window.steering_metrics.text()
data["output"]["steering_feedback"]["fresh"] = False
window.update_data(data)
assert "等待" in window.steering_metrics.text()
window.close()
window.deleteLater()
app.processEvents()
app.quit()
'''
    environment = dict(os.environ, QT_QPA_PLATFORM="offscreen")
    result = subprocess.run([sys.executable, "-c", script], env=environment,
                            text=True, capture_output=True, timeout=30)
    assert result.returncode == 0, result.stdout+result.stderr
