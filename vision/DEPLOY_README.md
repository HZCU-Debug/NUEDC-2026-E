# 树莓派部署与双摄像头配置

适用源码提交：`2725521` 之后的当前打包版本。目标平台为 Raspberry Pi 4B/5，建议使用
64位 Raspberry Pi OS。正式参数为 `1280x720`、`30 fps`、顺时针旋转 `180°`，串口
默认为 `/dev/serial0`、`115200 8N1`。

## 1. 安装发布包

先在开发机的 Windows PowerShell 中上传 ZIP。下面以树莓派用户 `debug`、地址
`192.168.1.89` 为例：

```powershell
scp "C:\Users\77019\Desktop\vision\releases\vision_rpi_2026-08-04_dual_camera_wayland.zip" debug@192.168.1.89:/home/debug/
```

如果目标机使用 `debug` 或其他用户名，必须同时修改 `用户名@IP` 和远端 `/home/用户名/`
路径。例如先确认能够登录：

```powershell
ssh debug@192.168.1.89
```

上传完成后，SSH登录树莓派并执行：

```bash
rm -rf /home/debug/vision
mkdir -p /home/debug/vision
unzip -o /home/debug/vision_rpi_2026-08-04_dual_camera_wayland.zip -d /home/debug/vision
cd /home/debug/vision
chmod +x install_rpi.sh run_rpi.sh focus_camera.sh verify_rpi.sh
./install_rpi.sh
```

删除旧目录后，摄像头需要重新执行Mode0标定。

安装脚本会安装 OpenCV、NumPy、pyserial、v4l-utils 和 C++/OpenMP 编译环境，创建
`.venv`，并在树莓派本机编译 `vision_fast`。`.so/.pyd` 与平台和Python ABI绑定，禁止
从Windows复制编译产物到树莓派。

如果当前用户不能访问串口：

```bash
sudo usermod -aG dialout "$USER"
```

随后退出SSH重新登录或重启。

## 2. 选择摄像头

设备选择文件是 `rpi_device_profiles.json`。它会同时选择：

1. 摄像头UVC参数文件；
2. 该摄像头自己的几何标定文件。

查看可用设备：

```bash
./run_rpi.sh --list
```

旧USB摄像头：

```bash
./run_rpi.sh old_usb
```

RaspberryPi-A/Microdia摄像头：

```bash
./run_rpi.sh raspberrypi_a
```

默认设备是 `old_usb`。也可以使用环境变量：

```bash
CAMERA_PROFILE=raspberrypi_a ./run_rpi.sh
```

两套内置映射：

| 设备名 | UVC参数 | 几何标定 |
|---|---|---|
| `old_usb` | `camera_profiles/old_usb_camera.json` | `calibrations/old_usb_camera.json` |
| `raspberrypi_a` | `camera_profiles/raspberrypi_a_camera.json` | `calibrations/raspberrypi_a_camera.json` |

### 2.1 新摄像头的统一初始参数

所有新摄像头先使用：

```text
camera_profiles/raspberrypi_a_camera.json
```

当前统一初始值来自 `raspberrypi06_fixed_clean`：

```json
{
  "white_balance_automatic": 1,
  "auto_exposure": 1,
  "exposure_dynamic_framerate": 0,
  "exposure_time_absolute": 120,
  "gain": 0,
  "brightness": 0,
  "contrast": 32,
  "saturation": 64,
  "hue": 0,
  "gamma": 100,
  "power_line_frequency": 1,
  "sharpness": 3,
  "backlight_compensation": 1
}
```

这些是摄像头驱动的UVC控制值，不是Mode0几何标定数据。不同驱动对同一个数字的定义
可能不同，例如 `auto_exposure=1` 必须以该摄像头 `v4l2-ctl` 的菜单说明为准。

### 2.2 在哪里修改摄像头参数

树莓派解压到 `~/vision` 后，新摄像头参数文件为：

```bash
nano ~/vision/camera_profiles/raspberrypi_a_camera.json
```

旧摄像头参数文件为：

```bash
nano ~/vision/camera_profiles/old_usb_camera.json
```

Nano底部显示的 `^O`、`^X` 中，`^` 表示键盘的 `Ctrl` 键。基本操作如下：

1. 使用方向键移动光标；
2. 直接删除并输入需要修改的数字，注意保留JSON中的双引号、冒号和逗号；
3. 按 `Ctrl+O` 保存；
4. 底部出现 `File Name to Write` 时不要修改文件名，直接按回车；
5. 出现 `Wrote ... lines` 表示保存成功；
6. 按 `Ctrl+X` 退出Nano。

如果修改错了并且不想保存：按 `Ctrl+X`，出现 `Save modified buffer?` 时按 `N`。如果
退出时希望保存，则按 `Y`，再按回车确认原文件名。

开发机仓库中的对应位置是：

```text
C:\Users\77019\Desktop\vision\camera_profiles\raspberrypi_a_camera.json
C:\Users\77019\Desktop\vision\camera_profiles\old_usb_camera.json
```

修改后先检查JSON格式：

```bash
python3 -m json.tool ~/vision/camera_profiles/raspberrypi_a_camera.json >/dev/null
```

程序只在启动时写入参数，因此保存后必须退出并重新执行：

```bash
cd ~/vision
./run_rpi.sh raspberrypi_a
```

### 2.3 怎么安全修改摄像头参数

先停止视觉程序，再确认当前摄像头支持的控制名、范围和菜单值：

```bash
v4l2-ctl -d /dev/video0 --list-ctrls-menus
```

只允许把设备实际支持、并且位于 `min..max` 范围内的项目写入JSON。`controls` 中的项目
默认是严格项，写入或回读失败会阻止正式启动；确实允许某型号缺失的项目才能加入
`optional_controls`。

现场试参时可以先不改JSON，临时写入单个值：

```bash
v4l2-ctl -d /dev/video0 --set-ctrl=gain=0
v4l2-ctl -d /dev/video0 --set-ctrl=exposure_time_absolute=120
```

回读确认：

```bash
v4l2-ctl -d /dev/video0 --get-ctrl=gain
v4l2-ctl -d /dev/video0 --get-ctrl=exposure_time_absolute
```

确认画面正确后再把数值写回JSON。建议一次只调整一类变量：先曝光/增益，再白平衡，
最后才调整亮度、对比度、gamma和锐度。不要同时大幅修改多个参数。

不要只切换UVC参数而继续使用另一台摄像头的标定。摄像头、镜头、支架、工作台、分辨率、
旋转角或成像参数改变后都必须重新标定。

## 3. 首次标定

发布包不包含任何会自动生效的现场标定。首次选择某台摄像头时，如果对应标定文件不
存在，程序会等待ESP32 Mode0：

1. 启动该设备，例如 `./run_rpi.sh raspberrypi_a`；
2. ESP32可靠发送一次Mode0，进入标定；
3. 放置平整、完整可见的 `105x297 mm` 湖蓝色半张A4；
4. 等待至少连续两帧角点稳定；
5. ESP32再次发送Mode0；
6. 程序保存到该设备自己的 `calibrations/*.json` 并立即热加载；
7. 移走蓝卡，再发送Mode1～4。

标定参考文件 `rpi_camera_calibration.reference.json` 仅供查看结构，不能改名冒充现场
标定。

## 4. 对焦与相机检查

```bash
./focus_camera.sh old_usb
./focus_camera.sh raspberrypi_a
```

检查设备身份和支持的控制项：

```bash
v4l2-ctl --list-devices
v4l2-ctl -d /dev/video0 --all
v4l2-ctl -d /dev/video0 --list-ctrls-menus
v4l2-ctl -d /dev/video0 --list-formats-ext
lsusb
```

同一UVC数值在不同型号摄像头上不等价。不要把旧摄像头的 `sharpness=70`、
`zoom_absolute` 等参数复制给只支持 `sharpness=0..6` 的Microdia摄像头。

## 5. 正式运行与预览

正式比赛不使用浏览器预览，使用树莓派Wayland桌面的本地窗口。先进入项目并设置
Wayland环境：

```bash
cd /home/debug/vision
export XDG_RUNTIME_DIR=/run/user/$(id -u)
export WAYLAND_DISPLAY=wayland-0
export QT_QPA_PLATFORM=wayland
```

新摄像头正式运行：

```bash
./run_rpi.sh raspberrypi_a
```

它会加载 `camera_profiles/raspberrypi_a_camera.json` 和该设备自己的Mode0标定文件，
不会添加 `--web`。等价的完整手工命令是：

```bash
python3 rpi_realtime_detection.py \
  --camera 0 \
  --width 1280 \
  --height 720 \
  --fps 30 \
  --rotate 180 \
  --detect-every 3 \
  --solver-workers 3 \
  --best-effort-after-seconds 90 \
  --camera-controls-file camera_profiles/raspberrypi_a_camera.json \
  --calibration-file calibrations/raspberrypi_a_camera.json \
  --serial-device /dev/serial0 \
  --serial-baud 115200
```

以前使用的 `--skip-camera-controls` 会跳过全部JSON参数，只适合临时排障。如果新摄像头
要使用本项目规定的统一初始参数，正式命令中不能添加该选项。

`--best-effort-after-seconds 90` 是比赛防卡死时限：从收到ESP32的Mode命令开始计时。
90秒内仍优先反复采集新稳定帧并求正确解，同时保存每次失败前评分最好的近似候选。
候选先比较已匹配碎片数，再比较已匹配接缝数，最后比较求解器评分。因此即使四块中
只有三块关系正确，也会优先发送这类能获得部分拼图分的方案，而不是重新随意摆放。
到时仍未成功，程序发送90秒内累计最好的候选，并在不改变其拼接关系的前提下加入
8 mm间隙、消除重叠并放入目标半区。JSON会显示 `best_effort=true`、
`puzzle_solved=false` 和 `best_effort_strategy=best_scored_partial_puzzle`。只有90秒内
没有产生任何可安全执行的求解候选时，才使用 `safe_compact_transport` 末级兜底。
默认值已写入
`rpi_device_profiles.json`，如需改为75秒，只修改：

```json
"best_effort_after_seconds": 75
```

四种模式都启用该截止机制：

| 模式 | 超时前累计的候选 |
|---|---|
| Mode1 | RMS误差最小的固定模板分配 |
| Mode2 | 匹配碎片和接缝最多、几何评分最好的矩形候选 |
| Mode3 | 匹配碎片和接缝最多、几何评分最好的扑克几何候选 |
| Mode4 | 接缝、双轴花纹和综合评分最好的被拒牌面候选 |

注意：`--manual` 是人工调试模式，不会自动触发90秒输出。若相机标定失败，或画面中
始终没有2～4块碎片的可信抓取坐标，程序不能向机械臂发送伪造坐标；应检查固定标定、
相机位置、画面边界和绿底分割。这个限制与“求解器有候选但最终验证失败”不同，后者
会正常进入最佳近似候选兜底。

旧摄像头运行：

```bash
./run_rpi.sh old_usb
```

额外参数可以放在设备名后面。例如改用 `/dev/ttyUSB0`：

```bash
./run_rpi.sh old_usb --serial-device /dev/ttyUSB0
```

启动日志必须检查：

```text
Native acceleration: enabled
Fixed calibration loaded: ...
Camera controls profile: ...
```

显示 `fallback` 时程序仍能运行，但Mode2/3会明显变慢。没有固定标定时，Mode1～4会
安全拒绝并等待Mode0，不会输出未经标定的机械坐标。

## 6. 安装后验证

```bash
./verify_rpi.sh
```

它会检查Python模块、C++扩展、两个设备配置和启动命令，不会打开摄像头或串口。

## 7. 增加第三种摄像头

1. 复制一个 `camera_profiles/*.json`，只保留该摄像头真实支持的控制项；
2. 在 `rpi_device_profiles.json::devices` 增加设备名；
3. 为它指定独立的 `calibrations/<设备名>.json`；
4. 执行 `./run_rpi.sh <设备名>`；
5. 使用Mode0完成该设备现场标定。

详细相机排障见 `CAMERA_AND_CALIBRATION.md`，状态机和串口流程见 `STATE_MACHINE.md`。
