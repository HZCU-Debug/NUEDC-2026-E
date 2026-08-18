# Repository Guidelines

## 项目说明

本仓库是用于全国大学生电子设计竞赛的电机控制项目，包含 ESP32 固件、树莓派视觉程序和 Python 上位机。ESP32 固件基于 Arduino 框架和 PlatformIO，上位机使用 uv 管理。

## 项目结构

- `firmware/`：ESP32 固件、PlatformIO 配置和电机 SDK。
- `host/`：Python 手柄、串口测试和树莓派视觉仿真程序。
- `vision/`：树莓派拼图识别、求解、相机配置和串口程序。
- `docs/`：操作指南、协作规则、赛题和硬件参考资料。

## 常用命令

- `pio run -d firmware -e esp32s3`：编译 ESP32-S3 N16R8 固件。
- `pio run -d firmware -e esp32`：编译旧 ESP32 固件。
- `pio run -d firmware -e esp32s3 -t upload`：编译并烧录 ESP32-S3 N16R8 固件。
- `pio run -d firmware -e esp32 -t upload`：编译并烧录旧 ESP32 固件。
- `./firmware/build.sh <esp32|esp32s3>`：更新指定控制板的编译数据库并烧录固件。
- `pio device monitor -d firmware`：打开串口监视器。
- `pio run -d firmware -e esp32s3 -t compiledb`：更新 clangd 使用的编译数据库，旧板开发时使用 `esp32` 环境。
- `pio run -d firmware -t clean`：清理固件构建产物。
- `uv sync --project host`：安装 Python 上位机依赖。
- `uv run --project host host/test_main.py`：运行 Python 上位机测试。
- `bash vision/install_rpi.sh`：在树莓派上安装视觉程序依赖并构建可选原生加速。
- `bash vision/run_rpi.sh --list`：列出视觉端设备配置。
- `bash vision/run_rpi.sh <old_usb|raspberrypi_a>`：按摄像头配置启动视觉程序。
- `bash vision/verify_rpi.sh`：检查视觉端依赖、原生加速和设备启动命令。

修改 ESP32 固件后分别运行 `pio run -d firmware -e esp32s3` 和 `pio run -d firmware -e esp32`。修改 Python 上位机时还需运行对应测试。修改树莓派视觉程序后运行 `bash vision/verify_rpi.sh`；涉及相机、识别或求解行为时还需在目标设备和代表性图片上验证。不要提交 `.pio/`、`compile_commands.json`、视觉运行输出和设备现场标定文件等生成内容。

`host/main.py` 是树莓派视觉端的串口仿真。除图像采集和求解外，它与 `vision/rpi_realtime_detection.py`、`vision/esp32_puzzle_link.py` 对 ESP32 呈现的 Mode 0～3 消息格式、发送顺序、可靠投递和运行生命周期必须一致；模拟数据可以不同，但必须满足固件接受的业务约束。视觉端的 Mode 4 尚未接入当前 ESP32 菜单和 Host 仿真。修改树莓派与 ESP32 之间的通信行为时，同步更新 Host 仿真及其测试。

# Repository Rules

1. 任何模块对外暴露的接口，需要进行注释
2. 注释避免使用行尾注释，对于接口使用 Doxgen 风格块注释 `/* */`，对于过程性代码使用行注释 `//`
3. 注释末尾禁止使用句号

## Agent skills

### Issue tracker

任务与需求使用 GitHub Issues；外部 PR 不作为需求分类入口。详见 `docs/agents/issue-tracker.md`。

### Triage labels

使用 `needs-triage`、`needs-info`、`ready-for-agent`、`ready-for-human`、`wontfix` 五种分类标签。详见 `docs/agents/triage-labels.md`。

### Domain docs

采用单一领域上下文，使用根目录 `CONTEXT.md` 和 `docs/adr/`。详见 `docs/agents/domain.md`。
