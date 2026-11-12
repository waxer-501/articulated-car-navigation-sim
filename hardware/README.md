# 独立铰接小车实机：双 IMU 无线回传

GitHub 维护入口为 `articulated-car-navigation-sim/hardware/`。本地历史文件夹名“清扫小车实机”沿用；本模块服务于独立铰接小车。首次克隆后在 `hardware/` 运行 `bash 安装实机环境.sh`；需构建固件及 ST-Link 旁路工具时运行 `bash 安装实机环境.sh --with-firmware-tools`。安装只创建局部环境和下载校验后的依赖，不连接、烧录或复位硬件。依赖恢复脚本支持已验证的 Ubuntu 22.04 x86_64；树莓派接收软件及正式部署仍待单独验证。

软件测试命令为 `/usr/bin/python3 -m unittest discover -s tests -v`。离线查看已有实测记录可运行 `bash 启动IMU监视.sh --offline`。GitHub 中的原始实测证据位于 `validation/2026-11-05/`；监视器会在本地 `captures/` 缺少旧记录时使用这份只读归档。详见该目录说明和 `references/README.md`。

电脑端桌面监视已提供：运行 [`启动IMU监视.sh`](启动IMU监视.sh)，或在应用菜单搜索“铰接小车 IMU通信监视”。窗口分别显示 STM32 采集、无线发送和 USB 实收，支持三轴曲线、原始包、自动记录和明确标识的历史回放；不烧录、不复位、不控制车辆。见 [监视器使用说明](docs/IMU通信监视使用.md)。

**无线回传已通过首轮低速实测：真实双 IMU → F103 → 裸 24L01P → 电脑 USB RX，100/100 个完整包收到，两路各 50 包，CRC/SPI/UART/队列丢失计数均为零。** 每路无线更新约 0.83 Hz，UART 继续各约 25 Hz。首次烧录后的调试重连失败由用户完整断电上电恢复，没有重刷；根因未确定。发送窗口已自动结束，树莓派端及正式持续回传尚未验证。详见 [无线实测结果](docs/f103无线回传实测结果_20261105.md)。

**2026-11-05 历史 UART 阶段（随后完成无线验证）：接收诊断固件已按用户授权烧录，真实双 IMU 静止采集通过。** 前路 A10 / 后路 A3 均约 25 Hz，30 秒窗口内新增有效帧 744/745，新增 UART 错误和队列丢失均为零，未观测到重启。ST-Link 偶发读回不一致已通过重复一致性校验拒绝并保留日志；稳定复核证明程序镜像正确、前 4 KiB 以外 Flash 保留。无线未启用，树莓派回传未验证。详见 [本轮实测结果](docs/f103双IMU实测_20261105.md)。

当前采集板为用户所报 STM32F103C6T6A 最小系统板。两只 IMU 接 USART1/USART2，裸无线模块接 SPI1，外设使用板载 3.3V；详见 [当前接线说明](docs/stm32f103c6接线.md)。SWD 读到设备 ID 0x0412、Flash 32 KiB、页大小 1 KiB，工具报告 SRAM 10 KiB；与 C6 规格一致，不据此确认完整芯片后缀或 SRAM 实测容量。下面 MC02 接口审查为历史资料。

## 本次范围（2026-11-03 用户确认）

```text
前车体 IMU406 ── UART ─┐
                       ├─ STM32F103C6T6A ── 裸 24L01P ))) ((( USB RX 底板 ── 树莓派
后车体 IMU406 ── UART ─┘
```

本模块只采集并回传 IMU 数据。车辆行驶与转向已由树莓派另一条链路封装，本项目不控制、不接管、不改动该链路。不迁移仿真导航、相机识别或执行器控制代码。

**当前已完成真实双 IMU UART 采集及 F103 SPI→USB RX 的低速无线回传，并保留此前电脑端 TX→RX 台架证据。** 加上电脑监视器，共 70 项离线测试用例通过：

- IMU406 UART 流解析、校验和、带符号 BCD 和 SI 单位转换（Python）。
- 无 HAL 依赖的 IMU406 C 解码器，已用于 F103 接收诊断固件。
- 32 字节、仅数据上报的遥测格式；纯 C 编码与树莓派 Python 解码逐字节一致。
- 遥测包含前后 IMU 身份、独立采集序号、MCU UART 收帧时间、启动标识、CRC16；当前无线诊断版本每次启动发送约一分钟后自动关闭 RF。
- 树莓派接收状态逻辑：重复包、乱序、丢包间隙、重启、缺失和过期数据处理。
- USB RX 串口采集入口和 CRC 字节流重组；支持分帧、粘帧和损坏后重新同步。

2026-11-04 用户确认 USB 底板分别为固定 TX 与 RX 角色。实测 RX 在 9600、8N1 下接收：3/3 自定义 ASCII 标记原样通过；分段间隔 300 ms 时，20/20 个合成遥测包、640 字节全部通过。50 ms 间隔的连续测试失败，原始结果保留。详见 [USB 无线实测记录](docs/usb_radio_20261104.md)。

F103C6 双 UART、SPI 无线驱动、启动编号日志及低速无线回传已通过本轮台架验证。树莓派实际接收部署、正式持续发送频率和长期稳定性仍未验证。电脑两个 USB 无线串口的台架记录保持原结论；没有发送传感器配置或车辆运动指令。

## 资料来源与核实结果

- 用户提供的 `3754002714IMU406 惯性测量单元.pdf`，V1.5，SHA-256 `7f33ac06adef02df57198683295157a5f1f33d4fdfa25456a593cdd9a9273b14`。
- 用户指定 [damiao_mc02_bsp](https://github.com/yssickjgd/damiao_mc02_bsp)，已取得原样只读工作副本 `vendor/damiao_mc02_bsp`，提交 `30ff2965d55afbcad500027b0e1e5031ea6f93b8`。
- 原芯片参考：[Nordic nRF24L01+ 产品规格书](https://docs-be.nordicsemi.com/bundle/nRF24L01P_PS_v1.0/raw/resource/enus/nRF24L01P_PS_v1.0.pdf)。仅作为候选芯片资料；尚不能据此确认用户模块、电源转接板或 `24L01P-RX` 的具体实现。

IMU406 手册中的明确接口：

| 项目 | 手册证据 |
|---|---|
| 电源、逻辑 | 3.3 V DC、3.3 V TTL；不能把 5 V 供电直接接到 IMU 的 3.3 V 脚 |
| UART 参数 | 默认 115200、8 数据位、无校验、1 停止位 |
| IMU 引脚 | 1=RX1，7=TX1，10=3.3V，12=GND |
| 保留接口 | 手册将 MOSI/MISO/SCK/NSS、CTX/CRX、TX3/RX3 等列为保留，不据此开发 SPI/CAN 采集 |
| 数据类型 | 姿态角、三轴加速度、三轴角速度；所谓“9 轴输出”不是磁力计证明 |
| 数据帧 | `68 1F address 84` + 27 字节 BCD 数据 + 1 字节校验和，共 32 字节 |
| 默认输出速率 | 协议页写默认 25 Hz；性能表列至 100 Hz，协议表另有 200 Hz 设置码，需实物确认 |

手册自身还有一处校验示例不一致：`68 05 00 8F 00 14` 按手册求和规则应为尾字节 `94`。代码遵守求和规则，不为错误示例绕过校验。工厂可定制输出格式；非 `0x84/0x1F` 格式会被明确拒绝，不能错当成九个有效测量值。

## 历史 MC02 接口审查（已停止采用）

这是指定 BSP 的**当前配置事实**，不是已经确认的实物接线：

| UART | MCU RX / TX | 例程状态 |
|---|---|---|
| USART1 | PA10 / PA9 | 115200 8N1，候选 TTL 接口 |
| UART7 | PE7 / PE8 | 115200 8N1，候选 TTL 接口 |
| USART10 | PE2 / PE3 | 115200 8N1，候选 TTL 接口 |
| USART2 / USART3 | PD6/PD5、PD9/PD8 | 例程走 RS485 配置，不能把外部 A/B 口当 3.3 V TTL |
| UART5 | PD2 / PC12 | 例程为 SBUS 接收配置，不直接用于 IMU |

无线 SPI 还不能定脚：SPI2 已用于板载 BMI088，SPI6 用于 WS2812；PE10/PE12 是板载 BMI088 中断线，并非空闲 GPIO。后续需根据实际板卡版本、外部引出和已接线情况选择独立 SPI、CE、CSN。

`dm02_test` 包含 CAN 电机初始化和周期发送，不用于本项目直接烧录。后续从 `dm02_test_pure` 准备独立工作副本，并裁剪无关外围初始化；“pure”本身也不等于已经完成无副作用启动审查。此次没有修改上游代码。

本机在 `.toolchain/` 准备局部 ARM GCC 10.3.1，未全局安装。当前 F103 接收诊断固件已独立构建、烧录并完成真实双 UART 采集；没有采用 MC02 的 H7 启动与驱动代码。

## 已实现文件

- `firmware/common/nrf24_tx.c`、`boot_journal.c`：射频发送状态机与双页启动编号日志。
- `firmware/f103_radio/radio_app.c`：新板 SPI1 绑定、最新帧轮询和有限发射窗口。
- `tools/verify_f103_radio.py`：默认仅预览，明确执行后备份、烧录、采集与独立整包统计验收。

- `sweeper_hw/imu406.py`：UART 完整帧与流解析。
- `sweeper_hw/inspect_capture.py`：离线文件/十六进制检查，不打开串口。
- `sweeper_hw/telemetry.py`：树莓派侧可复用的数据包解码和接收状态逻辑，只有标准库依赖。
- `sweeper_hw/usb_receiver.py`：USB RX 读取入口，仅接收；需 `pyserial`，不向 RX 写数据。
- `sweeper_hw/telemetry_stream.py`：从串口字节流恢复 32 字节 CRC 数据包。
- `firmware/common/imu406_decode.c`：MCU 可用的纯 C 解码内核。
- `firmware/common/telemetry_v1.c`：从有效 IMU406 帧直接编码遥测包，保留手册分辨率。
- [telemetry_v1 数据约定](docs/telemetry_v1.md)。

安装方向未确认，因此输出保持在各 IMU 自身坐标系。不会默认将前后坐标轴对齐，也不会把厂商输出航向当成绝对航向。

## 离线验证与使用

在本开发电脑、本目录运行；不连接任何硬件也能完成：

```bash
/usr/bin/python3 -m unittest discover -s tests -v
```

测试只在临时目录用 `cc` 编译纯 C 库及假 ST-Link 后端，并做软件字节流测试；车辆、IMU、无线电和 MCU 均不会被访问。通常不足 1 秒；`Ctrl+C` 仅终止本地测试。共 70 项用例；五项只读调试后端测试需要局部 stlink 开发头文件，缺少时明确跳过，其余测试不依赖硬件。测试覆盖协议、CRC、BCD、丢字节恢复、收包状态、F103 SRAM 诊断格式、不一致调试读回拒绝和监视器数据隔离，不能代替设备通信验证。

检查手册中的应答字节（只打印解析结果，不发送）：

```bash
/usr/bin/python3 -m sweeper_hw.inspect_capture --hex '68 05 00 8C 00 91'
```

已有原始 UART 录制文件时：

```bash
/usr/bin/python3 -m sweeper_hw.inspect_capture --file captures/front_uart.bin
```

该入口拒绝字符设备；它不会把命令行路径当串口打开。解析出的时间戳为 `null`，因为原始 IMU 帧没有时间戳，不能从离线文件凭空推算采样时刻。

## 已补充的信息与下一步

用户已改用一块 F103C6 并完成两只 IMU 接线和首次真实采集，USB 底板为 TX/RX 分工。2026-11-03 只读检查确认树莓派为 Pi 5 Model B Rev 1.1、Debian 13、Python 3.13.5，SPI 当时未启用。

当前按用户选择先接通真实 IMU，最终频率后定；先保留 USB RX 做低速验证。当前 32 字节/只的格式不可能通过 9600、8N1 的 UART 承载两路各 100 Hz。尚未修改树莓派 SPI、GPIO 或已有移动控制链路。
