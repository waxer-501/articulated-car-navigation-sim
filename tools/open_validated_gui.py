"""Open the real desktop UI, run a 120-second soak, then leave it paused for review."""
import json
import time
from PySide6.QtCore import QTimer
from PySide6.QtGui import QFont
from PySide6.QtWidgets import QApplication
import PySide6
from artic_nav.config import ROOT, load_config
from artic_nav.gui import Window


def main():
    output = ROOT/"runs/gui_soak"
    output.mkdir(exist_ok=True)
    app = QApplication([])
    app.setFont(QFont("Noto Sans CJK SC", 10))
    window = Window(load_config())
    window.show()
    started = time.monotonic()
    updates = []
    window.engine.updated.connect(lambda item: updates.append(item["output"]["time"]))
    timer = QTimer()
    def check():
        elapsed = time.monotonic()-started
        if elapsed < 120 or not updates:
            return
        result = {"passed": len(updates) >= 100 and window.engine.isRunning(),
                  "wall_seconds": elapsed, "updates": len(updates), "sim_seconds": updates[-1],
                  "pyside_version": PySide6.__version__, "display": "native X11"}
        (output/"result.json").write_text(json.dumps(result, indent=2))
        window.pause_button.setChecked(True)
        window.grab().save(str(output/"desktop.png"))
        window.send("export")
        print(json.dumps(result), flush=True)
        timer.stop()
    timer.timeout.connect(check)
    timer.start(500)
    app.exec()


if __name__ == "__main__":
    main()
