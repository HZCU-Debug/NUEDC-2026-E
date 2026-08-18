# 设备专用固定标定

每个JSON必须对应一套明确的物理组合：摄像头、镜头、支架姿态、工作台、分辨率、
`--rotate` 和最终相机profile。不同设备不要共用同一个现场标定文件。

建议命名：

```text
raspberrypi06_microdia_1280x720_rot180.json
raspberrypi4_oldusb_1280x720_rot180.json
```

正式运行仍默认读取工作目录中的 `rpi_camera_calibration.json`；部署时从本机对应档案
复制，或直接通过 `--calibration-file` 指定。发布包只提供reference文件，不应覆盖
目标机现场标定。

标定应在最终比赛相机参数和实际照明下进行。移动摄像头、支架或纸面后必须重做Mode0。
详细流程见 [`../CAMERA_AND_CALIBRATION.md`](../CAMERA_AND_CALIBRATION.md)。
