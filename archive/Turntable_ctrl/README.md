# Lift Platform Control Archive

这个目录用于归档历史 `Turntable_ctrl` 目录中的升降台控制源码，仅作为保存和参考，不参与当前 `mobile_robot_ws` 的 ROS 2 构建、安装或运行环境。目录名沿用历史命名，但当前固定串口语义是升降台，不是转盘。

设计约束：

- 目录放在工作空间根目录的 `archive/` 下，而不是 `src/`
- 提供 `COLCON_IGNORE`，避免被 `colcon` 误扫入构建
- 不复制原目录中的 `.venv`、`.pio`、`.vscode`、`__pycache__`、日志等环境和缓存文件

当前归档内容以原始目录结构为主，主要包含：

- `src/python_control(2)/python_control/motor_can_control.py`
- `src/python_control(2)/python_control/motor_can_gui.py`
- `src/python_control(2)/python_control/run_motor_can_gui.bat`
- `src/test pulse.py`
- `src/test_pulse.py`

说明：

- `src/SF_Control/` 当前只保留了源目录中的 `.gitignore`；原目录里未发现可直接归档的固件源码文件，构建产物 `.pio/` 已排除
- 如后续需要把这部分功能真正接入 ROS，请单独新建包，不要直接在本归档目录上做工作空间集成
