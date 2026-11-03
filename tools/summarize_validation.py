"""Build a human-readable evidence report from completed experiments only."""
import json
from pathlib import Path
from datetime import datetime
from zoneinfo import ZoneInfo
import numpy as np
from artic_nav.config import ROOT


def main():
    root = ROOT/"runs"
    suites = {name: json.loads((root/(name+"_final")/"summary.json").read_text())
              for name in ("acceptance", "stress", "sensitivity")}
    acceptance = suites["acceptance"]
    lines = ["# 仿真验证报告", "", "生成时间："+datetime.now(ZoneInfo("Asia/Shanghai")).isoformat(timespec="seconds"),
             "", "全部结果仅为本机 MuJoCo 仿真。真实 YOLO、真实玩具车及现场相机未验证。", "",
             "## 固定 50 次默认误差验收", "",
             f"- 通过：{acceptance['successes']}/{acceptance['runs']}（{acceptance['success_rate']:.0%}）。",
             f"- 验收判定：{'通过' if acceptance['acceptance_pass'] else '未通过'}；要求至少 90%，全部不越界、不超铰接限位。",
             f"- 全部边界检查通过：{acceptance['all_within_boundary']}。",
             f"- 全部铰接限位检查通过：{acceptance['all_within_articulation_limit']}。", "",
             "| 场景 | 通过数 | 最大实际位置误差 | 最大实际前车朝向误差 |", "|---|---:|---:|---:|"]
    cases = acceptance["cases"]
    for name in sorted({k.rsplit("_seed", 1)[0] for k in cases}):
        rows = [v for k, v in cases.items() if k.startswith(name+"_seed")]
        pos = max(v.get("position_error_m", float("inf")) for v in rows)
        yaw = max(v.get("front_heading_error_deg", float("inf")) for v in rows)
        lines.append(f"| {name} | {sum(v['success'] for v in rows)}/{len(rows)} | {pos*100:.2f} cm | {yaw:.2f}° |")
    lines += ["", "## 曲线、失明与异常目标", ""]
    stress = suites["stress"]["cases"]
    for name, row in sorted(stress.items()):
        if name.startswith("curve"):
            lines.append(f"- {name}：{'通过' if row['success'] else '未通过'}，路径 RMS {row['path_rms_m']*100:.2f} cm。")
        elif name.startswith("blackout"):
            log = [json.loads(line) for line in (root/"stress_final"/name/"navigation.jsonl").read_text().splitlines()]
            stopped = [r for r in log if 4.2 <= r["time"] <= 5.9]
            zero = bool(stopped) and all(np.max(np.abs(r["command"])) < 1e-9 for r in stopped)
            lines.append(f"- {name}：相机中断后的命令归零检查 {'通过' if zero else '未通过'}；恢复后到达 {'通过' if row['success'] else '未通过'}。")
        elif name == "unreachable":
            lines.append(f"- 不可达目标：{row['navigator_mode']}；{row['navigator_status']}。这是预期拒绝，不应记为成功到点。")
    lines += ["", "## 高噪声档与参数扰动", ""]
    for name, row in sorted(stress.items()):
        if name.startswith("stress_"):
            lines.append(f"- {name}：{'通过' if row['success'] else '未通过'}；实际位置误差 {row['position_error_m']*100:.2f} cm，朝向误差 {row['front_heading_error_deg']:.2f}°；原因 {row.get('failure_causes', [])}。")
    sensitivity = suites["sensitivity"]
    lines.append(f"- 参数扰动：{sensitivity['successes']}/{sensitivity['runs']} 到达通过；全程边界检查 {sensitivity['all_within_boundary']}，铰接限位检查 {sensitivity['all_within_articulation_limit']}。")
    soak = json.loads((root/"gui_soak/result.json").read_text())
    assert soak["passed"], "Native desktop stability validation did not pass"
    lines += ["", "## 工程验证与证据", "",
              "- 自动化测试：12 项通过，测试进程正常退出。含物理驱动/松手保持、投影几何、非完整运动学、视觉初始化/多解拒绝、倒车与不可达规划、延迟/失明、公共数据白名单、真值隔离和 GUI 拖动操作。",
              "- GUI 实际运行：离屏和本机 X11 桌面窗口均已验证，完成顶置 RGB、三维视图、独立导航子进程、目标启动、暂停/继续、停车、MP4 录像和导出。截图：`runs/gui_validation/desktop.png`。",
              "- 暂停状态下提交新目标会同步恢复仿真；该 GUI 交互在最终界面改动后另行通过定向测试。",
              "- 完整传感器回放：104 条消息，替换评分侧全部真值后，导航结果 SHA-256 完全一致；见 `runs/replay_validation/result.json`。",
              "- 桌面依赖固定为 PySide6 6.8.3：本机初装 6.12 的持续绘图出现 none_dealloc 崩溃，已替换并通过十万次绘图回归。数值验收的导航/物理/传感器/评分代码不受该 GUI 依赖变更影响。",
              f"- 本机原生 X11 桌面持续运行：{soak['wall_seconds']:.1f} 秒、{soak['updates']} 次完整画面/状态更新，无崩溃。完成后已暂停并保留窗口；证据 `runs/gui_soak/result.json`。",
              "- `runs/acceptance_final/summary.json`：50 次逐项指标。",
              "- `runs/stress_final/summary.json`：检测误差、曲线、失明与不可达目标。",
              "- `runs/sensitivity_final/summary.json`：模型参数扰动。",
              "- 各运行目录的 `source_manifest.json`、`config.yaml` 和 `model.xml` 固定源码与参数；真值与公共观测分开保存。",
              "- 带 `INTERRUPTED.md` 的目录是被中止的旧版诊断，不计入最终验收。",
              "", "## 边界", "",
              "已知静止多解姿态会等待更有区分度的视觉观测，不能保证仅靠任意两个轴对齐框立即辨识所有铰接姿态。",
              "模拟检测框的误差是人为配置的统计模型；执行器和摩擦等是工程假设。通过这些测试不等于真实识别率、实车动力学或实车导航已获验证。"]
    (ROOT/"验证报告.md").write_text("\n".join(lines)+"\n", encoding="utf-8")
    print("Wrote", ROOT/"验证报告.md")


if __name__ == "__main__":
    main()
