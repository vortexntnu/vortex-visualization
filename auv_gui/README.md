## AUV GUI
This package contains a graphical user interface (GUI) made for an AUV. Has both a mission and internal status interface.

Can be launched using the ROS 2 launch file:
```bash
ros2 launch auv_gui gui.launch.py
```

If you want to mock internal status data you can do:
```bash
ros2 launch auv_gui gui.launch.py mock_data:=true
```