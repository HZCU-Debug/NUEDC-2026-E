# NUEDC 电机控制

本仓库包含 ESP32 电机固件、树莓派视觉程序和 Python 上位机调试工具

## 项目结构

```text
firmware/   PlatformIO 固件、SDK 和测试
host/       Python 手柄和树莓派视觉仿真调试程序
vision/     树莓派拼图识别、求解和串口程序
docs/       赛题和硬件参考资料
```

## ESP32 固件

固件使用 1.14 英寸、240×135、ST7789V 彩屏显示菜单。当前提供以下 Demo 菜单项：

| 菜单项 | 用途 |
| --- | --- |
| `Controller Motor` | 接收 Python 手柄速度消息并控制电机 |
| `Motor Ramp` | 每 100 ms 改变 10 RPM，在 -300 至 300 RPM 之间往返 |
| `Motor Position` | 失能电机，每 500 ms 显示和输出转子角度 |
| `Comm Unreliable` | 每 500 ms 发送一次非可靠计数消息 |
| `Comm Reliable` | 可靠发送计数消息，确认后等待 500 ms 再发送下一条 |
| `Mock` | 接收 Host 固定数据并执行搬运任务 |
| `Sol 1` / `Sol 2-1` / `Sol 2-2` | 请求对应树莓派视觉赛题并执行搬运任务 |

使用 S1/S2 上下移动，S3 进入 Demo，S4 返回菜单。固件支持两套控制板配置：

| PlatformIO 环境 | 控制板 | 说明 |
| --- | --- | --- |
| `esp32` | ESP32 Dev Module | 旧控制板 |
| `esp32s3` | ESP32-S3 N16R8 | 默认控制板 |

### 编译期硬件配置

固件发布前在 `firmware/include/config/` 中调整配置：

| 文件 | 配置内容 |
| --- | --- |
| `hardware.h` | 控制板引脚、按钮上拉方式、电机组合、地址、方向和串口参数 |
| `display.h` | 屏幕型号、尺寸和旋转方向 |
| `parameters.h` | 各轴速度、加速度、到位误差、查询间隔和校准参数 |

`hardware.h` 中的 `kButtonsUseInternalPullup` 为 `false` 时按钮使用外部上拉和 `INPUT`，为 `true` 时使用 ESP32 内部上拉和 `INPUT_PULLUP`。

通过 `kMotorLayout` 选择电机组合：

| 配置值 | X/Y 轴 | Z/Roll 轴 |
| --- | --- | --- |
| `MotorLayout::AllZdt` | 张大头 | 张大头 |
| `MotorLayout::AllAtk` | 正点原子 | 正点原子 |
| `MotorLayout::Mixed` | 张大头 | 正点原子 |

编译期根据组合装配对应的电机适配器。`esp32-s3-cbh` 的张大头电机使用 `GPIO40/41`；正点原子电机使用 `GPIO43` 发送，Z 和 Roll 轴分别使用 `GPIO44`、`GPIO42` 接收；视觉串口使用 `GPIO12/13`。屏幕使用 `display.h` 中声明的 `Display` 类型，并通过 `kDisplay` 设置宽度、高度和旋转方向。

分别编译两套固件：

```shell
pio run -d firmware -e esp32
pio run -d firmware -e esp32s3
```

烧录时选择实际连接的控制板环境：

```shell
pio run -d firmware -e esp32 -t upload
pio run -d firmware -e esp32s3 -t upload
```

`firmware/build.sh` 会为指定控制板更新编译数据库并烧录固件：

```shell
./firmware/build.sh esp32
./firmware/build.sh esp32s3
```

电机 Demo 使用 `Serial2` 连接原理图的 `RXD1` 和 `TXD1`。进入 `Motor Position` 后可通过串口监视器查看角度：

```shell
pio device monitor -d firmware
```

`Controller Motor` 使用 `comm::Link` 接收消息类型 `1` 的速度命令。载荷是一个 2 字节大端有符号整数：

```text
消息类型: 1
发送模式: 非可靠
载荷: int16 大端序
```

速度量范围为 `-1000` 到 `1000`，映射到 `-300` 到 `300 RPM`。无效类型、长度或数值会被忽略。连续 500 ms 没有收到有效命令时，电机自动停止。

启动流程等待电机上电 2 秒，发送使能命令并读取一次位置。USB 串口只承载二进制协议帧，不输出文本日志。

## Python 上位机

安装 [uv](https://docs.astral.sh/uv/) 后执行：

```shell
uv sync --project host
uv run --project host host/main.py --port /dev/cu.usbserial-0001
```

进入固件的 `Test` 后，可通过上位机逐条发送单轴速度或绝对位置命令：

```bash
uv run --project host host/motor_test.py --port /dev/cu.usbserial-0001
```

速度命令格式为 `轴名 RPM`，例如 `X 300` 或 `R -200`；位置命令格式为
`轴名 目标角度 最大RPM`，例如 `X 90 300`。轴名支持 `X`、`Y`、`Z`、`R`，
绝对角度以固件初始化时清零的位置为基准，发送 `X 0` 可用普通速度命令停止 X 轴

先运行 `main.py`，再进入固件的 `Mock`。ESP32 可靠发送模式 `1`，Host 确认题号后发送 4 块固定拼图数据：先发送数量消息，再逐块发送编号、当前位置、目标位置和旋转角度。所有消息使用可靠模式，收到 ESP32 确认后才发送下一条。ESP32 使用毫米坐标执行搬运，并将旋转角度直接用作 ID 4 电机的绝对角度。

`host/main.py` 仿真当前 ESP32 使用的 Mode 0～3 请求、结果消息顺序和可靠投递，只用固定数据代替相机识别结果。视觉端的自动重试、忙碌状态、重新允许接收命令、取消请求和 Mode 4 不在 Host 仿真范围内。三个正式赛题由 `vision/` 下的树莓派程序执行。

| 消息类型 | 大端序载荷 | 说明 |
| --- | --- | --- |
| `0x03` | `uint8 mode` | ESP32 请求视觉模式，当前整机使用 0–3 |
| `0x01` | `uint8 count` | 拼图数量，范围 1–4 |
| `0x02` | `uint8 id + 5 × int16` | 编号、当前 X/Y、目标 X/Y、0.01° 角度 |

手柄 Demo 默认读取第一个手柄的 axis 3，每 50 ms 向开发板发送一次速度命令。Windows 串口可以写为 `COM3`：

```shell
uv run --project host host/controller_demo.py --port COM3 --axis 3
uv run --project host host/controller_demo.py --port COM3 --axis 3 --invert
```

按 `Ctrl-C` 退出时会发送零速度命令

手柄采集行为参考 [ZhiGrip-Joystick](https://github.com/LanternCX/ZhiGrip-Joystick/blob/main/main.py)，串口通信使用 `host/link.py` 中与固件一致的 COBS 与 CRC-8 协议。

## 树莓派视觉端

视觉程序运行在 Raspberry Pi 4B/5，通过 USB 摄像头识别和求解拼图，再使用与 Host
仿真相同的可靠串口协议发送动作计划。首次部署在树莓派上执行：

```shell
cd vision
bash install_rpi.sh
bash run_rpi.sh --list
bash run_rpi.sh old_usb
```

`raspberrypi_a` 摄像头使用 `bash run_rpi.sh raspberrypi_a`。每个设备配置分别关联摄像头
参数和现场标定文件；移动摄像头、支架或纸面，或者修改正式成像参数后，需要重新执行
`Cal Vision`。在仓库根目录运行 `bash vision/verify_rpi.sh` 可检查依赖、可选原生加速和
两套设备的启动命令。

当前 ESP32 菜单与视觉模式的对应关系如下：

| ESP32 菜单 | Mode | 视觉任务 |
| --- | ---: | --- |
| `Cal Vision` | 0 | 两段式蓝卡固定标定 |
| `Sol 1` | 1 | 固定四块彩色拼图 |
| `Sol 2-1` | 2 | 未知纯色或白色碎片 |
| `Sol 2-2` | 3 | 不规则扑克牌碎片 |

视觉代码还包含用于矩形牌面花纹重建的 Mode 4，但当前 ESP32 菜单和 Host 仿真不会发送
该模式。自动求解会等待连续两帧几何稳定；正常求解持续失败时，默认在 90 秒后发送
期间得到的最佳安全方案。标定或抓取坐标不可信时不会发送伪造坐标。

部署、相机配置和状态机细节分别见
[`vision/DEPLOY_README.md`](vision/DEPLOY_README.md)、
[`vision/CAMERA_AND_CALIBRATION.md`](vision/CAMERA_AND_CALIBRATION.md) 和
[`vision/STATE_MACHINE.md`](vision/STATE_MACHINE.md)。

## 通信 Demo 接收端

进入 `Comm Unreliable` 或 `Comm Reliable` 后，运行同一个 Python 接收程序：

```shell
uv run --project host host/receive_counter.py --port /dev/cu.usbserial-0001
```

程序显示消息的可靠性和递增计数。可靠模式的确认由 `Link` 自动返回。

## 自定义通信协议

上位机和固件通过 `comm::Link` 协议双向通信。串口参数为 115200 baud、8N1。协议先组织原始包，再进行 COBS 编码，最后追加 `00` 作为帧结束符：

```text
线格式 = COBS(原始包) + 00
```

原始包分为三种：

| 包类型 | 原始包格式 |
| --- | --- |
| 非可靠消息 | `TYPE PAYLOAD CRC` |
| 可靠消息 | `TYPE_WITH_RELIABLE SEQ PAYLOAD CRC` |
| 确认包 | `00 SEQ CRC` |

字段定义：

| 字段 | 长度 | 说明 |
| --- | ---: | --- |
| `TYPE` | 1 字节 | 消息类型，范围 `01`–`7F` |
| `TYPE_WITH_RELIABLE` | 1 字节 | `TYPE | 80`，最高位表示可靠消息 |
| `SEQ` | 1 字节 | 可靠消息序列号，从 `00` 开始并在 `FF` 后回绕 |
| `PAYLOAD` | 可变 | 业务载荷，最大长度由通信端点的 `Link` 容量决定 |
| `CRC` | 1 字节 | 覆盖原始包中此前全部字节的 CRC-8 |

CRC-8 使用多项式 `07`、初始值 `00`、结果异或值 `00`，按最高位优先计算。对字符串 `123456789` 的校验结果为 `F4`。

例如，类型 `01`、载荷 `11 00 22` 的非可靠消息如下。载荷中的 `00` 由 COBS 编码处理，只有末尾的 `00` 用于分帧：

```text
原始包: 01 11 00 22 31
线格式: 03 01 11 03 22 31 00
```

类型 `01`、序列号 `07`、载荷 `44` 的可靠消息及其确认包如下：

```text
可靠消息原始包: 81 07 44 D0
可靠消息线格式: 05 81 07 44 D0 00
确认包原始包:   00 07 15
确认包线格式:   01 03 07 15 00
```

### 通信过程

非可靠消息只发送一次，接收端校验 COBS、CRC、消息类型和载荷长度后交给业务代码；无效包直接丢弃。

可靠消息采用单包等待确认的方式：

1. 发送端写入带 `SEQ` 的可靠消息并保存完整线格式帧
2. 接收端校验成功后立即返回相同 `SEQ` 的确认包
3. 发送端收到匹配的确认包后产生 `Delivered` 事件
4. 未收到确认时，发送端每 50 ms 重发一次原帧
5. 接收端仍会确认重复帧，但不会把与上一条相同 `SEQ` 的消息再次交给业务代码

等待确认期间，同一端点不能发送其他可靠或非可靠消息。协议没有最大重试次数；退出 Demo 或不再需要发送时，应调用 `cancel()` 停止重传。双方必须持续调用 `poll()` 才能处理接收、确认和重传。

该机制用于处理串口传输中的临时丢包，不提供持久化或严格的 exactly-once 保证。固件调用 `Link::begin()` 时会清空当前收发状态，但保留发送序列号和重复包记录；重新创建 `Link` 对象时从初始状态开始。

`Controller Motor` 和计数 Demo 使用类型 `01`；拼图通信使用类型 `03` 发送视觉模式、类型 `01` 发送数量、类型 `02` 发送单块数据。

## LSP

安装 clangd 和 PlatformIO 后生成固件编译数据库：

```shell
pio run -d firmware -e esp32s3 -t compiledb
```

在编辑器中重启 clangd 即可。使用旧控制板开发时将环境替换为 `esp32`。修改 `firmware/platformio.ini`、依赖或源文件后需要重新生成编译数据库
