## AUV GUI
This package contains a graphical user interface (GUI) made for an AUV. Has both a mission and internal status interface.

## Prerequisites
- [vortex_utils package](https://github.com/vortexntnu/vortex-utils)
- PyQt6, can be installed with `pip install PyQt6`
- pyopengl, can be installed with `pip install pyopengl`

## Launching
Can be launched using the ROS 2 launch file:
```bash
ros2 launch auv_gui gui.launch.py
```

If you want to mock internal status data you can do:
```bash
ros2 launch auv_gui gui.launch.py mock_data:=true
```
## Troubleshooting
If the GUI does not launch properly, this might help:
```bash
export QT_QPA_PLATFORM=xcb
```

If the GUI struggles with establishing an OpenGL context, and can't draw any objects, this might help:
```bash
export PYOPENGL_PLATFORM=glx
```
