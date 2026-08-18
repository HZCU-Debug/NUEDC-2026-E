# 相机、光照与固定标定指南

最后核对：2026-08-03

## 1. 三类问题必须分开

| 层级 | 典型现象 | 首要检查 |
|---|---|---|
| 相机打不开/不出帧 | `Cannot open /dev/video0`、`select() timeout` | 设备号、占用、USB、曝光与帧率组合 |
| 颜色或亮度异常 | 整幅偏粉、偏暗、噪声大 | 实际UVC范围、profile、MJPG/YUYV对照 |
| 几何标定异常 | Mode1 RMS大、Mode2尺寸/角点拒绝、机械坐标偏 | 本机Mode0标定、支架、纸面与光照 |

不要用放宽求解阈值来掩盖前两层问题。

## 2. 确认摄像头身份和能力

```bash
v4l2-ctl --list-devices
v4l2-ctl -d /dev/video0 --all
v4l2-ctl -d /dev/video0 --list-ctrls-menus
v4l2-ctl -d /dev/video0 --list-formats-ext
lsusb
```

控制项输出中的 `min/max/default/value` 分别是允许范围、默认值和当前实际值。配置文件
必须只包含设备支持的控制项；例如不支持 `zoom_absolute` 的相机不能使用含该项且未
标记optional的profile。

## 3. profile原则

- 老USB摄像头：`camera_profiles/old_usb_camera.json`；
- RaspberryPi-A/Microdia类摄像头：`camera_profiles/raspberrypi_a_camera.json`；
- 通过 `--camera-controls-file` 显式选择；
- `--skip-camera-controls` 只用于短时诊断；
- 修改profile后必须重启程序才会重新应用；现场实时试值可先用 `v4l2-ctl --set-ctrl`；
- 同样的曝光、增益、gamma数值在不同摄像头上不保证产生相同画面。

新摄像头统一从 `camera_profiles/raspberrypi_a_camera.json` 开始，当前初始档案名为
`raspberrypi06_fixed_clean`，控制值为：

```text
white_balance_automatic=1  auto_exposure=1
exposure_dynamic_framerate=0  exposure_time_absolute=120  gain=0
brightness=0  contrast=32  saturation=64  hue=0  gamma=100
power_line_frequency=1  sharpness=3  backlight_compensation=1
```

这份JSON是UVC成像参数，不是Mode0生成的几何标定。树莓派上用
`nano ~/vision/camera_profiles/raspberrypi_a_camera.json` 修改；保存后先运行
`python3 -m json.tool` 检查格式，再重启视觉程序。修改前必须用
`v4l2-ctl -d /dev/video0 --list-ctrls-menus` 核对当前驱动支持的控制名、范围和菜单含义。
可以先用 `v4l2-ctl --set-ctrl` 临时试值，再回写JSON；每次只调整一类参数并回读验证。
任何正式成像参数变化后，都要在最终光照下重新执行该设备自己的Mode0标定。

正式比赛预览使用Wayland本地窗口，不添加 `--web`。启动前设置：

```bash
export XDG_RUNTIME_DIR=/run/user/$(id -u)
export WAYLAND_DISPLAY=wayland-0
export QT_QPA_PLATFORM=wayland
```

新摄像头正式运行必须通过 `--camera-controls-file
camera_profiles/raspberrypi_a_camera.json` 加载统一初始参数；历史命令中的
`--skip-camera-controls` 会完全跳过该文件，只能用于判断“参数写入是否导致问题”的短时
排障，不能作为比赛启动命令。

建议先恢复相机默认或中性参数，确认颜色正常，再逐项锁定曝光和白平衡。不要同时大幅
修改曝光、增益、亮度、gamma和色温，否则无法定位哪个参数造成变化。

## 4. 洋红/偏粉画面

2026-08-02现场样例中，自动与手动白平衡画面均偏洋红，红蓝均值约比绿色高25%；
恢复中性控制后，MJPG与YUYV都恢复正常。因此该样例不是IR-cut损坏，也不是固定的
MJPG解码错误，而是旧profile组合与新摄像头不兼容。

推荐诊断步骤：

1. 停止所有视觉程序；
2. 拔插摄像头，读取默认值；
3. 使用 `brightness=0, contrast=32, saturation=64, hue=0, gamma=100,
   sharpness=3` 作为中性起点；
4. 自动曝光/白平衡对准真实绿色工作区稳定5～10秒；
5. 对比MJPG和YUYV原图；
6. 颜色恢复后再在真实工作区读取曝光与增益，并决定是否锁定。

自动白平衡开启时读取到的 `white_balance_temperature` 可能只是驱动保留值，不一定是
自动算法内部的真实通道增益，不能仅凭该数字锁定色温。

## 5. 暗画面与长曝光

静态题目允许牺牲帧率，优先使用“更长曝光、较低增益”，但必须满足驱动约束：

- `exposure_time_absolute=320` 约为32 ms，通常能接近30fps；
- `400` 约为40 ms，超过30fps单帧时间；若动态帧率关闭，部分固件会停止出帧；
- 需要400以上曝光时，先测试 `exposure_dynamic_framerate=1`；
- 增益越高，黑色花纹越容易产生彩色噪声并污染轮廓；
- 亮度和gamma是数字处理，不应代替真实曝光。

长曝光前先用独立流测试：

```bash
v4l2-ctl -d /dev/video0 \
  --set-fmt-video=width=1280,height=720,pixelformat=MJPG \
  --set-parm=30 --stream-mmap=3 --stream-count=30 --stream-to=/dev/null
```

## 6. 无法打开或读取帧超时

```bash
v4l2-ctl --list-devices
ls -l /dev/video*
fuser -v /dev/video0
```

- 有占用进程：先正常停止原程序；
- 设备号变化：按 `v4l2-ctl --list-devices` 使用真实Video Capture节点；
- 无占用但超时：恢复短曝光、拔插USB，必要时重载 `uvcvideo`；
- `No fixed calibration file yet` 与相机打不开是两个独立问题。

## 7. 固定标定

标定文件绑定摄像头、镜头、支架姿态、工作台、分辨率、旋转和最终成像配置。不同物理
设备即使型号相同，也建议各自Mode0标定。

标准流程：

1. 使用最终比赛profile启动；
2. 第一次Mode0进入标定；
3. 放置平整、完整可见的 `105×297 mm` 湖蓝半张A4；
4. 确认四角连续两帧稳定；
5. 第二次Mode0保存；
6. 移走蓝卡再测试Mode1和Mode2。

夜间LED照明可能同时影响蓝卡角点和运行时绿色/白色分割。若多套设备在同一时段共同
失败、另一时段共同恢复，优先做同一设备、同一摆法、不同照明的受控A/B，而不是把
问题归结为四台独立硬件同时损坏。

## 8. Mode1/2与标定的关系

Mode1允许碎片任意旋转和平移，但不允许任意缩放。标定比例、透视或轮廓错误会直接
增加模板RMS；当前最大单块阈值为 `8 mm`。

Mode2/3依赖毫米尺寸、直角、边覆盖、外角位置和接缝距离。坏标定可能导致安全拒绝，
也可能产生表面合法但机械坐标偏移的更危险结果。因此换相机、移动支架或改变分辨率后
必须重新标定。

## 9. 现场对比需要保存的文件

```text
rpi_solve_input.png
rpi_solve_detection.jpg
rpi_solve_detection.json
rpi_failure_*.png/.jpg/.json
rpi_failure_history.jsonl
rpi_camera_calibration.json
camera profile JSON
启动日志中的Native acceleration状态
```

比较两套设备时，代码、模板文件哈希应该一致；标定文件通常应该不同：

```bash
sha256sum fragment_vision.py vision_state_machine.py puzzle_solver.py fixed_puzzle_template.json
sha256sum rpi_camera_calibration.json
```
