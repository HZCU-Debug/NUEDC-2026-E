# 摄像头参数档案

- `old_usb_camera.json`：原EP20CC54K01类摄像头，包含 `sharpness=70` 和
  `zoom_absolute`。
- `raspberrypi_a_camera.json`：所有新摄像头的统一初始档案，名称为
  `raspberrypi06_fixed_clean`；初始采用自动白平衡、`exposure_time_absolute=120`、
  `gain=0`、`contrast=32`、`gamma=100`、`sharpness=3`，不包含
  `white_balance_temperature` 和 `zoom_absolute`。
- `old_usb_camera_low_gain_experimental.json`：旧摄像头低增益实验档案，不作为默认
  比赛参数。

正式启动通过 `rpi_device_profiles.json` 选择设备；选择会同时关联UVC参数和专用几何
标定路径。修改档案后必须重启程序，并在最终参数、最终光照下重新执行Mode0标定。

树莓派上修改新摄像头参数：

```bash
nano ~/vision/camera_profiles/raspberrypi_a_camera.json
python3 -m json.tool ~/vision/camera_profiles/raspberrypi_a_camera.json >/dev/null
v4l2-ctl -d /dev/video0 --list-ctrls-menus
```

JSON中的控制项必须是当前驱动真实支持的名称，数值必须在驱动报告的范围内。先用
`v4l2-ctl --set-ctrl=<名称>=<数值>` 临时试参并回读，确认后再写回JSON。参数只在程序
启动时加载，修改后必须重启；改变正式成像参数后还必须重新Mode0几何标定。
