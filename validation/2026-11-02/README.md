# 2026-11-02 仿真验收证据

从本机已完成的实验产物原样复制；这是历史验收记录，不是上传时重新执行 50 次实验的结果。

- [默认误差 50 次验收](acceptance_final/summary.json)：50/50 通过，无越界与铰接超限。
- [检测压力与故障场景](stress_final/summary.json)：含高噪声超差和预期不可达拒绝，保留失败记录。
- [参数扰动](sensitivity_final/summary.json)：8/8 通过。
- [真值隔离回放](replay_validation/result.json)：更换评分真值前后，导航输出哈希一致。
- [桌面持续运行](gui_soak/result.json)：原生 X11 窗口持续运行检查。
- [界面截图](gui_validation/desktop.png)。
- [文件校验值](sha256.json)。

原始传感器流、真值轨迹、MP4、运行环境和中断实验日志保留在原开发电脑，不纳入仓库。可按根目录 README 的命令生成新的实验记录；所需源码与依赖版本已上传。上述结果不代表真实 YOLO 或实车验证。
