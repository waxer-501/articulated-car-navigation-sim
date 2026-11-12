#!/usr/bin/env python3
"""Targeted repair of BRLTTY grabbing the two observed CH340 radio adapters.

Default is read-only. --apply needs local sudo. No serial port is opened and no
payload/configuration bytes are sent. USB topology is deliberately restricted.
"""
import argparse
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import time

PORTS = ("1-1.2", "1-1.4")
RULE = Path("/etc/udev/rules.d/85-brltty.rules")
ACCESS_RULE = Path("/etc/udev/rules.d/70-sweeper-radio-uaccess.rules")
VENDOR_RULE = Path("/lib/udev/rules.d/85-brltty.rules")
MARKER = "# SWEEPER RADIO: exclude the two verified CH340 USB ports only"


def run(*args, check=True):
    return subprocess.run(args, check=check, capture_output=True, text=True)


def devices():
    result = []
    for port in PORTS:
        path = Path("/sys/bus/usb/devices")/port
        vid = (path/"idVendor").read_text().strip() if (path/"idVendor").exists() else None
        pid = (path/"idProduct").read_text().strip() if (path/"idProduct").exists() else None
        driver = path/(port+":1.0")/"driver"
        result.append({"usb_port": port, "vid": vid, "pid": pid,
                       "driver": driver.resolve().name if driver.exists() else None})
    return result


def atomic_write(path, text):
    with tempfile.NamedTemporaryFile("w", dir=path.parent, delete=False) as stream:
        stream.write(text)
        stream.flush()
        os.fsync(stream.fileno())
        temp = Path(stream.name)
    temp.chmod(0o644)
    temp.replace(path)


def apply():
    if os.geteuid() != 0:
        raise SystemExit("需要本机 sudo 管理员认证；默认检查不需要权限。")
    found = devices()
    if any(d["vid"] != "1a86" or d["pid"] != "7523" for d in found):
        raise SystemExit("两个指定 USB 插口上的设备已变化，未进行修改。请重新识别。")
    original = RULE.read_text() if RULE.exists() else VENDOR_RULE.read_text()
    if 'LABEL="brltty_device_end"' not in original:
        raise SystemExit("BRLTTY 规则布局不符合已审查版本，未进行修改。")
    if MARKER in original:
        raise SystemExit("已安装此定向修复；请先检查设备状态，不重复覆盖。")
    if ACCESS_RULE.exists():
        raise SystemExit("专用访问规则已存在，未覆盖。")
    backup = Path("/var/backups")/("sweeper-radio-"+datetime.now().strftime("%Y%m%d-%H%M%S"))
    backup.mkdir(mode=0o700)
    state = {"files": {}, "ports": list(PORTS),
             "brltty_was_active": run("systemctl", "is-active", "brltty-udev.service", check=False).stdout.strip() == "active"}
    for path in (RULE, ACCESS_RULE):
        exists = path.exists() or path.is_symlink()
        if path.is_symlink():
            raise SystemExit("现有规则是符号链接，停止以保护原有配置。")
        state["files"][str(path)] = {"existed": exists}
        if exists:
            shutil.copy2(path, backup/path.name)
    guards = MARKER+"\n"+"\n".join(
        f'SUBSYSTEM=="usb", KERNEL=="{p}", ENV{{PRODUCT}}=="1a86/7523/*", GOTO="brltty_device_end"'
        for p in PORTS)+"\n"
    access = "# Grant the active desktop user access to these two radio serial ports.\n"+"\n".join(
        f'SUBSYSTEM=="tty", KERNELS=="{p}", ATTRS{{idVendor}}=="1a86", ATTRS{{idProduct}}=="7523", TAG+="uaccess"'
        for p in PORTS)+"\n"
    for path, content in ((RULE, guards+original), (ACCESS_RULE, access)):
        state["files"][str(path)]["installed_sha256"] = hashlib.sha256(content.encode()).hexdigest()
    (backup/"state.json").write_text(json.dumps(state, indent=2)+"\n")
    atomic_write(RULE, guards+original)
    atomic_write(ACCESS_RULE, access)
    run("udevadm", "control", "--reload-rules")
    # This is the udev-created service currently claiming these CH340 adapters.
    # Do not disable/uninstall BRLTTY or alter its regular service.
    run("systemctl", "stop", "brltty-udev.service")
    for port in PORTS:
        interface = port+":1.0"
        driver = Path("/sys/bus/usb/devices")/interface/"driver"
        if driver.exists() and driver.resolve().name == "usbfs":
            (driver/"unbind").write_text(interface)
        if not driver.exists():
            Path("/sys/bus/usb/drivers_probe").write_text(interface)
    run("udevadm", "settle", "--timeout=5", check=False)
    serials = []
    for tty in Path("/sys/class/tty").glob("ttyUSB*"):
        parts = (tty/"device").resolve().parts
        if not any(port in parts for port in PORTS):
            continue
        run("udevadm", "trigger", "--action=add", str(tty))
        serials.append("/dev/"+tty.name)
    run("udevadm", "settle", "--timeout=5", check=False)
    # uaccess is persistent for these connectors; grant the sudo invoker an
    # immediate ACL as well when setfacl is present (no global dialout change).
    uid = os.environ.get("SUDO_UID")
    if uid and uid.isdigit() and shutil.which("setfacl"):
        for serial in serials:
            run("setfacl", "-m", f"u:{uid}:rw", serial)
    print(json.dumps({"backup": str(backup), "devices": devices(), "serials": serials,
                      "radio_payload_sent": False}, ensure_ascii=False, indent=2))
    if len(serials) != 2:
        raise SystemExit("规则已安装，但两路串口尚未全部恢复；保留输出和备份以继续排查。")


def restore(backup):
    if os.geteuid() != 0:
        raise SystemExit("恢复配置需要 sudo。")
    backup = Path(backup)
    state = json.loads((backup/"state.json").read_text())
    if set(state["files"]) != {str(RULE), str(ACCESS_RULE)}:
        raise SystemExit("非本工具备份，拒绝恢复。")
    for name, info in state["files"].items():
        path = Path(name)
        if path.is_symlink() or not path.exists() or hashlib.sha256(path.read_bytes()).hexdigest() != info["installed_sha256"]:
            raise SystemExit("安装后规则发生变化；不覆盖其他修改。")
    for name, info in state["files"].items():
        path = Path(name)
        if info["existed"]:
            shutil.copy2(backup/path.name, path)
        else:
            path.unlink()
    run("udevadm", "control", "--reload-rules")
    print("原有规则已恢复；设备驱动重新识别可通过手动拔插完成。")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--apply", action="store_true")
    mode.add_argument("--restore", metavar="BACKUP_DIRECTORY")
    args = parser.parse_args()
    if args.apply:
        apply()
    elif args.restore:
        restore(args.restore)
    else:
        print(json.dumps({"devices": devices(), "read_only": True}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
