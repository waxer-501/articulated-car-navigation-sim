"""Live offscreen GUI smoke: physics, child process, recording, pause and export."""
import os
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
import json
from pathlib import Path
from PySide6.QtWidgets import QApplication
from PySide6.QtCore import QTimer
from PySide6.QtGui import QFont
from artic_nav.config import load_config,ROOT
from artic_nav.gui import Window

def main():
 app=QApplication([]); app.setFont(QFont('Noto Sans CJK SC',10)); window=Window(load_config()); window.show()
 evidence=ROOT/'runs/gui_validation'; evidence.mkdir(exist_ok=True)
 flags=set()
 def check():
  d=window.last_data
  if d is None: return
  t=d['output']['time']
  if t>1.5 and 'target' not in flags:
   window.selected([.1,-.45,0.]); window.camera.target=window.target; window.start_goal(); flags.add('target')
   window.record_button.setChecked(True)
  if t>2 and 'pause' not in flags:
   window.send('pause',True); flags.add('pause'); QTimer.singleShot(300,lambda:window.send('pause',False))
  if t>4 and 'done' not in flags:
   flags.add('done'); window.grab().save(str(evidence/'desktop.png'))
   window.record_button.setChecked(False); window.send('export'); window.send('stop')
   (evidence/'session.txt').write_text(d['directory'])
   QTimer.singleShot(2000,finish)
 def finish():
  window.engine.stopping=True
  window.engine.wait(5000)
  window.close(); app.quit()
 timer=QTimer(); timer.timeout.connect(check); timer.start(100)
 QTimer.singleShot(120000,finish)
 app.exec()
 assert 'done' in flags,flags
 print(json.dumps({'checks':sorted(flags),'screenshot':str(evidence/'desktop.png')}))
if __name__=='__main__':main()
