# Python Control

这个目录当前包含 3 个可直接使用的启动文件：

- `run_motor_can_gui.bat`
- `motor_can_gui.py`
- `motor_can_control.py`

## 1. 启动图形界面

推荐直接运行批处理文件：

```powershell
.\run_motor_can_gui.bat
```

这个脚本会自动：

- 切换到当前目录
- 查找 Python 3
- 检查并安装依赖 `python-can` 和 `pyserial`
- 启动 `motor_can_gui.py`

如果不想通过 `.bat` 启动，也可以直接运行：

```powershell
python .\motor_can_gui.py
```

Linux 下直接运行：

```bash
python3 motor_can_gui.py
```

GUI 会优先选择 `/dev/ttyACM2`。如果只输入 `ttyACM2`，程序会自动转换成 `/dev/ttyACM2`。

## 2. 启动命令行控制脚本

`motor_can_control.py` 是命令行版本，必须带子命令启动。

常用命令如下：

```powershell
python .\motor_can_control.py diagnose
python .\motor_can_control.py enable
python .\motor_can_control.py disable
python .\motor_can_control.py monitor
python .\motor_can_control.py move --position 1.0 --kp 20 --kd 1
```

## 3. 当前建议的使用方式

如果你想先确认程序能跑起来，建议按下面顺序：

1. 启动 GUI：

```powershell
.\run_motor_can_gui.bat
```

2. 如果要排查底层通信，再运行命令行诊断：

```powershell
python .\motor_can_control.py diagnose
```

## 4. 说明

- 当前磁盘中确认存在的文件只有 `run_motor_can_gui.bat`、`motor_can_gui.py`、`motor_can_control.py`。
- 编辑器标签页里如果出现 `run_motor_can_gui(1).bat`、`motor_can_gui(1).py`，但磁盘里没有这些文件，说明它们可能还未保存，未保存时无法直接运行。
