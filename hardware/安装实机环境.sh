#!/bin/bash
# Creates only the project virtual environment; never opens or flashes hardware.
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
if [[ ! -x .venv/bin/python ]]; then
    /usr/bin/python3 -m venv --without-pip .venv
fi
task_site=$(.venv/bin/python -c 'import site; print(site.getsitepackages()[0])')
/usr/bin/python3 -m pip install --disable-pip-version-check --target "$task_site" -r requirements.txt
.venv/bin/python -c 'import serial, tkinter; print("pyserial", serial.VERSION, "Tk", tkinter.TkVersion)'
if [[ ${1:-} == --with-firmware-tools ]]; then
    /usr/bin/python3 tools/fetch_toolchain.py
    /usr/bin/python3 tools/fetch_f103_cmsis.py
fi
echo 'Environment prepared. No hardware was opened; run 启动IMU监视.sh to monitor.'
