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

On receiving this error:
```bash
qt.qpa.wayland: eglSwapBuffers failed with 0x300d, surface: 0x0
```
The following might fix it:
```bash
export QT_QPA_PLATFORM=xcb
```
