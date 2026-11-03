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
window.close()
window.deleteLater()
app.processEvents()
app.quit()
'''
    environment = dict(os.environ, QT_QPA_PLATFORM="offscreen")
    result = subprocess.run([sys.executable, "-c", script], env=environment,
                            text=True, capture_output=True, timeout=30)
    assert result.returncode == 0, result.stdout+result.stderr
