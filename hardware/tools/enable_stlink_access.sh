#!/bin/bash
# Install USB access only; never connect to, reset, erase or flash the MCU.
set -euo pipefail
if [[ $EUID -ne 0 ]]; then
    echo 'Run with sudo; no administrator password should be sent in chat.' >&2
    exit 1
fi
task_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source_rule="$task_root/configs/60-sweeper-stlink.rules"
target_rule=/etc/udev/rules.d/60-sweeper-stlink.rules
if [[ -L "$target_rule" ]]; then
    echo "Refusing to replace a symbolic link: $target_rule" >&2
    exit 1
fi
if [[ -f "$target_rule" ]] && ! cmp -s "$source_rule" "$target_rule"; then
    backup_dir=$(mktemp -d /var/backups/sweeper-stlink-XXXXXXXX)
    cp -p -- "$target_rule" "$backup_dir/60-sweeper-stlink.rules"
    echo "Previous rule saved to $backup_dir"
fi
install -m 0644 -- "$source_rule" "$target_rule"
udevadm control --reload-rules
udevadm trigger --action=add --subsystem-match=usb --attr-match=idVendor=0483 --attr-match=idProduct=3748
echo 'ST-Link/V2 USB permissions installed. No MCU command was sent.'
