#!/bin/bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
mkdir -p captures
exec 9>captures/.imu_monitor.lock
if ! flock -n 9; then
    echo 'IMU 通信监视窗口已在运行，请使用已有窗口。'
    exit 0
fi
exec .venv/bin/python -m sweeper_hw.monitor "$@"
