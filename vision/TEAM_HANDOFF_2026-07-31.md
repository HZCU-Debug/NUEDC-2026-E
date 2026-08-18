# 当前项目交接（更新于2026-08-03）

> 文件名因历史引用保留，但内容已更新为当前工作树，不再代表2026-07-31 stable.4。

## 1. 版本状态

- 当前版本父基线：`1eb9ca0 Accelerate puzzle solving and harden Mode 2`；
- 当前版本包含后续 Mode2/3 矩形与间隙修改、Mode3反向绿色分割、Mode2四矩形
  专用布局、测试和实验相机配置；
- 这些改动形成新的本地提交后，本地 `main` 将领先本地记录的 `origin/main`；未经
  用户要求不要推送；
- 发布目录和ZIP是诊断/部署生成物，不代表Git发布状态。

## 2. 当前能力

- Mode0：湖蓝色 `105×297 mm` 半张A4固定标定；
- Mode1：固定四块模板匹配，最大单块 RMS 阈值 `8 mm`；
- Mode2：未知纯色碎片的矩形几何拼接；
- Mode3：不规则扑克碎片，按几何拼接，不执行牌面花纹终审；
- Mode4：直角矩形扑克碎片，执行角标、双轴和180°牌面验证；
- ESP32可靠串口：COBS、CRC-8、逐包ACK、重复包处理和 `0xFF` 取消；
- `vision_fast` C++/OpenMP可选加速，实时默认 `--solver-workers 3`。

## 3. 当前几何口径

- 规范纸面为 `210×297 mm`，`4 px/mm`；
- 源区：`Y>148.5 mm`；目标区：`Y<148.5 mm`；
- 拼图整体最终距中线 `10 mm`；
- 所有模式从无间隙几何开始，先验证原始拼接，再增加间隙；
- `8 mm` 是相邻真实接缝边的法向距离，不是任意顶点之间的距离；
- 增加间隙后只要求接缝距离、零重叠和工作区合法，不再拿加缝后的外接尺寸验证原始
  矩形大小。

Mode2/3的矩形回退包括：

- 严格外轮廓四角；
- 强矩形角度回退：其他证据一致时允许手工切边产生的较大单角误差；
- 带缝外边回退：不要求栅格并集只有四个角，而是将真实碎片边投影到矩形四侧；
- 带缝回退要求四侧均有边、最小夹角 `>=80°`、最小覆盖率 `>=0.25`、平均覆盖率
  `>=0.60`，并继续检查尺寸、填充率、连接性和匹配误差。

## 4. Mode3反向绿色分割

`fragment_vision.py::build_piece_mask()` 在 Mode3 下启用
`include_dark_artwork_in_piece_mask`：

1. 从当前矫正帧学习绿色背景色相；
2. 依据色相差、最低饱和度和最低亮度建立背景掩膜；
3. 直接反相得到非绿色卡片；
4. 按背景亮度比例 `artwork_dark_value_ratio=0.58` 保守补回深色牌面像素；
5. 明确移除中线后再提取碎片轮廓。

不要把绿色范围固定写死成 `H=35..85`：历史图片背景色相可能高于85。也不要恢复大
范围的灰色像素扩张，实验中它会破坏历史样例。当前历史Mode3抽样14/14输出；现场
梅花问题帧仍因后续几何不合法而安全拒绝，是尚未解决的第二层问题。

## 5. Mode2/3搜索调度

- 角驱动 Beam=120；Mode2角容限18°，Mode3为35°；
- 角快通道只有分数 `<=0.147` 且通过全部最终安全检查才提前采用；
- T形或长边内部接缝回退完整边/分段边；
- Mode3全为近似直角四边形时跳过角通道，避免对称换位；这类扑克通常应使用Mode4；
- Mode2常规 Beam 为40、80；
- Mode3实时先尝试40、400，连续失败两张新稳定帧后才增加1600；
- 自动稳定帧同时检查数量、顶点数、质心、顶点和边长。

## 6. 相机与标定现状

相机配置必须按型号分离：

- `camera_profiles/old_usb_camera.json`：老摄像头，包含 `sharpness=70` 和
  `zoom_absolute`；
- `camera_profiles/raspberrypi_a_camera.json`：Microdia类摄像头，`sharpness=6`，
  不包含 `zoom_absolute`；
- `camera_profiles/old_usb_camera_low_gain_experimental.json`：老摄像头低增益实验档，
  尚不能视为所有设备的比赛默认值。

2026-08-02/03现场经验：同样的UVC数值在不同摄像头上不等价。将老参数中的
`gamma=300`、长曝光、手动白平衡等组合套到另一台摄像头时曾出现严重洋红色；恢复
中性默认值后，MJPG和YUYV颜色都正常，说明不是固定的视频格式故障。长曝光400且
`exposure_dynamic_framerate=0` 在 `1280×720@30` 上还可能造成 `select() timeout`。

不同设备必须各自保存 `rpi_camera_calibration.json`。同一标定文件不应复制到相机、
镜头、安装姿态或工作台不同的设备。标定应在最终相机配置和最终现场光照下完成。

昨晚多套设备Mode1/2失败、次日另一套设备快速成功的根因尚未完全闭环。当前优先
假设是共享环境变量（夜间LED照明、蓝卡分割、轮廓质量、共同错误标定），其次检查
是否加载原生扩展。不要把该现象写成已确认的单一硬件故障。

详细排障见 [`CAMERA_AND_CALIBRATION.md`](CAMERA_AND_CALIBRATION.md)。

## 7. Mode4状态

- 只接受2～4块近似直角矩形牌片；
- 临时几何重叠最多 `15 mm²`，最终必须零重叠；
- 几何候选上限60，槽位候选上限48，角标锚定最多128，完整评分批量16；
- 数字/小花色角标必须位于右上和左下；
- 四块模式要求 `min(long_axis, short_axis)>=0.60`；
- 长轴相对最佳候选保留窗口0.04；
- 拼缝形态学完全停用，`seam_diagnostics_computed=false`；
- `PASS` 不是人工真值，必须目视检查solution图。

## 8. 当前测试基线

2026-08-03当前工作树：

```text
原生路径：58 passed
强制回退：54 passed, 4 skipped
```

命令：

```powershell
& 'D:\Anaconda\python.exe' -m pytest -q
$env:VISION_DISABLE_NATIVE='1'
& 'D:\Anaconda\python.exe' -m pytest -q
```

修改Mode4仍需回归 `test3_42..47、52、57` 并目视检查，`test3_55`应继续安全拒绝。
修改Mode3分割需至少回归 `test3_13、17、39、40、58..67`。

## 9. 部署注意

- 同步Python、JSON、`setup_fast.py` 和 `native/`；
- 不同步Windows编译出的 `.pyd/.so`，树莓派本机执行
  `python3 setup_fast.py build_ext --inplace --force`；
- 默认不覆盖目标机 `rpi_camera_calibration.json`；
- 启动检查 `Native acceleration: enabled`、固定标定已加载、相机profile名称正确；
- `/home/debug` 与 `/home/debug1` 是不同用户目录，命令必须按当前登录用户名调整；
- 压缩包 `vision_rpi_2026-08-01_mode3_inverse_green_checkpoint.zip` 是诊断检查点，包含
  反向绿色分割，但不包含现场标定，也不包含后来现场临时创建的相机JSON。

## 10. 失败诊断

失败时优先保留：

```text
rpi_solve_input.png
rpi_solve_detection.jpg
rpi_solve_detection.json
rpi_failure_raw.png
rpi_failure_background_mask.png
rpi_failure_divider_mask.png
rpi_failure_overlay.jpg
rpi_failure.json
rpi_failure_history.jsonl
rpi_camera_calibration.json
实际使用的camera profile
```

Mode1出现 `最大RMS 22 mm` 级错误时，优先检查错误标定、轮廓缺失、夜间光照和模板
文件版本，不要将 `8 mm` 阈值直接放宽。Mode2/3看到 `outer=6` 不代表一定拼错；有
实际间隙时栅格并集会产生额外角点，应结合四侧边证据判断。
