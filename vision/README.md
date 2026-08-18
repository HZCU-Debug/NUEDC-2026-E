# 树莓派拼图视觉系统

最后核对：2026-08-03

本项目运行在 Raspberry Pi 4B/5，使用 USB 摄像头识别绿色 A4 工作纸上的碎片，
计算抓取坐标、目标坐标和旋转角，并通过可靠串口协议把完整动作计划发送给 ESP32。

当前代码支持 Mode0 标定和 Mode1～4 求解。完整开发上下文见
[`AGENTS.md`](AGENTS.md)，现场相机和标定排障见
[`CAMERA_AND_CALIBRATION.md`](CAMERA_AND_CALIBRATION.md)。

## 坐标与工作区

- A4 左上角为 `(0, 0)`；X 向右 `0..210 mm`，Y 向下 `0..297 mm`；
- 中线为 `Y=148.5 mm`；规范坐标下半区是物理右侧源区，上半区是物理左侧目标区；
- 输出角度顺时针为正，规范化到 `[-180°, 180°)`；
- 拼装完成后，整体距中线 `10 mm`；
- 矩形合法性在增加间隙前检查；随后相邻真实接缝边沿法向分开 `8 mm`；
- 最终输出要求零重叠、目标全部在合法工作区内。

## 四种模式

| 模式 | 任务 | 当前策略 |
|---|---|---|
| 1 | 固定四块彩色拼图 | 与 `fixed_puzzle_template.json` 做刚性轮廓匹配；单块最大 RMS 必须 `<=8 mm` |
| 2 | 未知纯色/白色碎片 | 角驱动快通道、完整边/分段边 Beam、矩形几何与带缝外边证据 |
| 3 | 不规则扑克牌碎片 | 自适应反向绿色分割、几何搜索、比 Mode2 更宽的搜索调度 |
| 4 | 2～4块直角矩形扑克碎片 | 矩形规范化、右上/左下角标、双轴与180°花纹验证 |

Mode3 只按不规则扑克碎片的几何关系拼接；Mode4 才负责矩形牌面的花纹重建。

### Mode2/3 当前几何规则

- Mode2恰好检测到四块高置信度矩形时，先枚举横/竖切分、2×2和T形全矩形布局；
  该通道只要求形成任意合法矩形，不恢复原始标签顺序，失败时自动回退通用边Beam；
- 两者共享最终矩形校验，不把最终 `8 mm` 间隙带入矩形尺寸判断；
- Mode2 拼前长边范围 `90..120 mm`，Mode3 为 `84..120 mm`，短边均为
  `50..90 mm`；
- 常规外轮廓角优先要求接近四个直角；手工裁切或实际拼缝产生额外角点时，可使用
  四条外侧边的覆盖率和相邻边夹角进行安全回退；
- 带缝边回退要求四侧都有证据、最小外角至少 `80°`、最小单边覆盖率 `0.25`、
  平均覆盖率 `0.60`，同时继续检查尺寸、填充率、连接性和匹配误差；
- 候选通过矩形校验后，才将真实相邻接缝边沿法向分开 `8 mm`。

### Mode3 当前分割

Mode3 不再把“白色”直接等同于碎片，而是学习当前绿色背景色相并做反向分割：

```text
绿色背景掩膜 -> 取反得到卡片 -> 补回保守的深色牌面像素
```

这样黑色花纹触碰裁切边时不容易把真实轮廓向内挖掉。绿色色相是每张图自适应学习，
没有固定写死为 HSV `[35,50,50]..[85,255,255]`，因为不同摄像头和曝光下绿色可能
超过 `H=85`。当前历史 Mode3 图片抽样 `test3_13/17/39/40/58..67` 为 14/14 输出；
现场梅花问题帧虽然轮廓已有改善，但几何搜索仍安全拒绝，尚不能声称完全解决。

## 主要文件

| 文件 | 用途 |
|---|---|
| `rpi_realtime_detection.py` | 正式入口：相机、稳定帧、预览、ESP32控制和自动重试 |
| `fragment_vision.py` | 固定标定、A4矫正、背景分割、碎片轮廓和抓取点 |
| `vision_state_machine.py` | 四种模式、动作计划、间隙和最终安全检查 |
| `puzzle_solver.py` | Mode1模板匹配及Mode2/3几何搜索 |
| `card_pattern_solver.py` | Mode4矩形牌面像素评分 |
| `esp32_puzzle_link.py` | COBS、CRC-8、ACK、重发和消息打包 |
| `rpi_camera_controls.py` | 按相机JSON写入并回读UVC控制项 |
| `native/vision_fast.cpp` | Mode2/3/4几何热点的C++/OpenMP加速 |
| `setup_fast.py` | 构建可选的 `vision_fast` 原生扩展 |

## 安装与原生编译

```bash
cd ~/vision
sudo apt update
sudo apt install -y v4l-utils build-essential python3-dev python3-opencv python3-numpy python3-serial
python3 setup_fast.py build_ext --inplace --force
python3 -c "import vision_fast; print(vision_fast.__file__)"
```

启动日志应检查：

```text
Native acceleration: enabled
```

显示 `fallback` 时仍可运行，但 Mode2/3 可能明显变慢。编译后的 `.so/.pyd` 与平台和
Python ABI 绑定，必须在树莓派本机编译，不能从 Windows 复制。

## 相机配置与固定标定

每种摄像头使用独立的 `camera_profiles/*.json`。同一个数值在不同UVC摄像头上的实际
效果可能完全不同；不要把老摄像头的 `zoom_absolute`、`sharpness=70` 等配置套到只
支持 `sharpness=0..6` 的摄像头上。

固定标定文件 `rpi_camera_calibration.json` 绑定：

- 具体摄像头和镜头；
- 支架位置、角度和高度；
- 工作台位置；
- 分辨率与 `--rotate`；
- 标定时使用的最终相机成像配置。

不同物理设备原则上分别执行两次 Mode0：第一次进入蓝卡标定，第二次在连续两帧角点
稳定后保存。不要把发布包中的 reference JSON 改名冒充现场标定。

Mode1 的 `固定模板匹配误差过大：最大RMS ...` 通常说明轮廓、毫米比例或透视异常，
不是碎片摆放位置和旋转不同；阈值当前为 `8 mm`，出现 `22 mm` 级误差时不应放宽。

## 正式运行

```bash
cd /home/debug/vision

export XDG_RUNTIME_DIR=/run/user/$(id -u)
export WAYLAND_DISPLAY=wayland-0
export QT_QPA_PLATFORM=wayland

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
  --serial-device /dev/serial0 \
  --serial-baud 115200
```

上面的用户名、目录和相机配置只是示例，必须按目标树莓派修改。正式比赛使用Wayland
本地预览，不添加 `--web`。`--skip-camera-controls` 只用于临时排障，新摄像头正式运行
必须加载 `camera_profiles/raspberrypi_a_camera.json`。

只有纯 SSH、没有本地Wayland窗口的临时排障环境才增加：

```bash
--web --port 8080
```

## ESP32可靠控制

- ESP32→Pi：`TYPE=0x03`，payload 为 `0/1/2/3/4`；`0xFF` 取消当前任务；
- Pi→ESP32：`TYPE=0x01` 发送碎片数；
- Pi→ESP32：`TYPE=0x02`，payload 为 `>Bhhhhh`；
- 坐标单位 mm，旋转单位 `0.01°`；
- 每包收到 ACK 后才发送下一包；
- 求解失败时保持当前任务、等待 `0.75 s`、重新采集两张几何稳定帧并重试；
- 每次失败都保留该帧评分最好的近似拼图，并跨重试比较；优先级依次为已匹配碎片数、
  已匹配接缝数、求解器评分；
- 从收到本次Mode命令开始累计到 `90 s` 仍无正常解时，发送90秒内最好的近似拼图，
  因此“三块关系正确、第四块错误”的候选会优先于无拼接关系的整齐摆放；该方案标记
  `best_effort=true`、`puzzle_solved=false`，并继续保证目标上半区、零重叠和 `8 mm`
  间隔；只有求解器从未产生可安全执行的候选时，才使用紧凑搬运末级兜底；
- 忙碌期间重复 Mode 不会打断机械任务，只有可靠 `0xFF` 会取消。

四种模式都启用同一90秒截止，但保存的近似候选不同：

| 模式 | 90秒内保存的最佳候选 |
|---|---|
| Mode1 | 固定模板分配中RMS误差最小的近似组合 |
| Mode2 | 匹配碎片/接缝最多、几何评分最好的矩形候选 |
| Mode3 | 匹配碎片/接缝最多、几何评分最好的扑克几何候选 |
| Mode4 | 接缝、双轴花纹和综合评分最好的被拒牌面候选 |

该机制只在正式自动任务中启用；使用 `--manual` 时不会自动计时或发送保底计划。若A4
标定失败，或始终无法得到2～4块碎片的可信抓取坐标，则不能编造串口坐标；程序只能
继续等待新画面或ESP32取消命令。固定标定的正式树莓派运行通常不会依赖离线图片中的
A4边界重新检测。

详见 [`STATE_MACHINE.md`](STATE_MACHINE.md)。

## 测试

```powershell
& 'D:\Anaconda\python.exe' -m pytest -q
```

2026-08-04 当前工作树验证结果：

```text
66 passed
```

强制关闭原生扩展：

```powershell
$env:VISION_DISABLE_NATIVE='1'
& 'D:\Anaconda\python.exe' -m pytest -q
```

结果：

```text
62 passed, 4 skipped
```

2026-08-04扩展图片抽测：Mode3的`test3_13/17/39/40`正常输出；Mode4的
`test3_42/46/50/52/57`正常并经目视检查，`test3_55`可导出最佳被拒候选，
`test3_56`无评分候选但可执行紧凑搬运末级兜底。`test3_54`当前存在错误放行：两个
数字角标落在同一侧，尚未修复；它不能计入正确回归。`test3_48/49`的A4边界超出画面，
离线自动标定失败且无可信抓取坐标。

图片回归不能只看 `valid=True`；必须检查 detection/solution 成图，并区分正确输出、
安全拒绝和错误放行。
