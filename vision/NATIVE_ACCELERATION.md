# 树莓派 C++ 原生加速

项目支持一个可选的 `vision_fast` C++ 扩展。扩展不存在或加载失败时，程序自动使用
Python/NumPy 实现，不影响四种 Mode 的正常运行。

当前原生实现：

- Mode 2/3/4 使用的真实外轮廓角点测量；
- Mode 2/3/4 几何搜索的整阶段凸多边形重叠批处理，可通过 OpenMP 使用多核；
- Mode 2/3/4 将整阶段的边对齐旋转、平移和移动碎片世界坐标变换合并为一次
  C++/OpenMP 批处理，避免 Python 为每个 Beam 分支重复执行小矩阵运算；
- Mode 2/3 大批量状态的可行性检查、量化变换签名和 Beam 预评分；
- Mode 4 Beam 先由原生分数预筛，再在截断边界恢复 Python/OpenCV 精确排序，
  因此原生与回退路径保持相同候选、评分和动作；
- 单张二值图的平移重叠内核（默认仍使用更快的 NumPy 实现）；
- 支持 OpenMP 的批量平移重叠接口，为后续整批候选评分使用。

## 树莓派安装与编译

```bash
cd /home/debug1/vision
sudo apt update
sudo apt install -y build-essential python3-dev
python3 setup_fast.py build_ext --inplace
```

编译完成后，当前目录应出现类似文件：

```text
vision_fast.cpython-3xx-arm-linux-gnueabihf.so
```

64位系统的文件名通常包含 `aarch64-linux-gnu`。

验证：

```bash
python3 - <<'PY'
import vision_fast
print("vision_fast loaded:", vision_fast.__file__)
PY
```

启动实时程序时会打印：

```text
Native acceleration: enabled
```

如果显示 `fallback`，程序仍能运行，但没有加载 C++ 扩展。

## 运行与回退

默认运行：

```bash
python3 rpi_realtime_detection.py \
  --camera 0 \
  --width 1280 \
  --height 720 \
  --fps 30 \
  --rotate 180 \
  --detect-every 3 \
  --solver-workers 3 \
  --serial-device /dev/serial0 \
  --serial-baud 115200
```

现场发现原生模块异常时，可强制回退：

```bash
VISION_DISABLE_NATIVE=1 python3 rpi_realtime_detection.py \
  --camera 0 --width 1280 --height 720 --fps 30 --rotate 180 \
  --detect-every 3 --serial-device /dev/serial0 --serial-baud 115200
```

`--solver-workers 3` 会并行 Mode 2/3 的 Beam 重叠检测和 Mode 4 的独立花纹评分。
四核树莓派建议先用 `3`，给相机采集、串口和系统保留一个核心；也可实测 `4`。

并行不会改变候选筛选顺序：原生内核按输入索引写回结果，Python仍按原稳定顺序
建立和排序候选。当前测试中1/3/4 workers的评分、拼接坐标、旋转角和动作一致；
仅 `plan_id`、耗时以及极小的浮点末位可能不同。

当前开发机抽样耗时（不同图片复杂度差异较大）：

- Mode2：约 `0.46～0.47 s`；
- Mode3：约 `0.32～2.76 s`；
- Mode4第二阶段优化后成功样例：约 `1.45～1.75 s`。

这些不是树莓派保证值，部署后必须用现场图片重新计时。

## 更新源文件后重新编译

```bash
cd /home/debug1/vision
python3 setup_fast.py build_ext --inplace --force
```

`.so/.pyd` 是本机生成物，不提交仓库，也不要在Windows和树莓派之间直接复制。
应当同步 `native/vision_fast.cpp` 与 `setup_fast.py`，然后在目标树莓派本机编译。

## 回归验证

```bash
python3 -m pytest -q
```

2026-08-03 当前工作树原生模式为 `54 passed`，包括 C++ 边对齐与 Python 参考实现
逐项等价测试，以及后续矩形/间隙回归。强制回退验证：

```bash
VISION_DISABLE_NATIVE=1 python3 -m pytest -q
```

回退结果为 `50 passed, 4 skipped`，跳过项是必须加载原生扩展才能比较的等价性测试。

历史图片回归见 `tmp/cpp_edge_batch_regression_2026-08-01/`。Mode3 的
test3_13/17/39/40、test2_9 与 Mode4 的 test3_42～47、52、57、41 均保持上一版
候选、评分和动作；test3_55 继续安全拒绝。Mode4 拼缝形态学已完全不执行，
`seam_diagnostics_computed=false`，Warp 源掩膜则在同次求解内复用。开发机受控 A/B
中原生边对齐路径明显快于对应 Python 路径；树莓派上的实际收益仍以现场计时为准。
