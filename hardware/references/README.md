# 资料与第三方依赖

本仓库保留用户提供的两份 IMU/USB 底板手册原文件，版权及厂商声明不变。维护代码不依赖复制的 MC02 H7 SDK。大型厂商包和重复解压副本按来源、版本与散列恢复，不作为项目源码重复提交。

- IMU406 V1.5：`../3754002714IMU406 惯性测量单元.pdf`，SHA-256 `7f33ac06adef02df57198683295157a5f1f33d4fdfa25456a593cdd9a9273b14`。
- AS06-VTB07H：`../3755724803AS06-VTB07H收发一体测试底板使用说明.pdf`，原文件内容保持不变。
- 泽耀 AS01-ML01D 原始资料包：<http://www.ashining.com.cn/data/as01_ml01d.zip>，SHA-256 `cc0ddf53418bc004351c4af4a4eef3cd2e47cdce4ebae2f9f48fa6497f8e110e`。
- 原 MC02 历史参考：<https://github.com/yssickjgd/damiao_mc02_bsp>，固定提交 `30ff2965d55afbcad500027b0e1e5031ea6f93b8`，只供追溯旧文档，不用于 F103 烧录。
- ST/ARM CMSIS：`../configs/f103_cmsis_sources.json` 锁定每个文件 URL 和 SHA-256，包含各自 License 原文；运行 `python3 tools/fetch_f103_cmsis.py` 恢复，第三方版权保持不变。
- ARM GCC 10.3.1、binutils 2.38、newlib 3.3、stlink 1.7.0 与 USB 开发头文件：`../configs/toolchain_packages.json` 锁定 Ubuntu 包版本及散列。运行 `python3 tools/fetch_toolchain.py`，只解压到 `.toolchain/`，不用 sudo，不改变系统工具链。

上述固定二进制工具已在 Ubuntu 22.04 x86_64 核验；不能将其直接部署到树莓派 ARM 系统。主机需有 `cc`、`dpkg-deb`、`python3-venv`、`python3-pip`、`python3-tk` 以及 libusb 运行库；缺少系统包时先按操作系统安装。安装脚本不自动改 udev、系统服务或进行硬件测试。

GitHub 中不包含 `.venv/`、`.toolchain/`、构建产物、原设备完整 Flash 备份或实时监视日志；实测报告保留镜像/原备份散列及验证结论。保存了用于回放和核对的原始无线字节、测量包及诊断记录。
