# URDF-models

This package contains the [URDF](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html) files for the robot model. The URDF files define the robot's physical structure, including links and joints.

## Foxglove Studio
In order for Foxglove studio to recognise the [meshes](./meshes/base_link.STL) the [foxglove bridge](https://foxglove.dev/docs/studio/connection/using-foxglove-bridge) needs be run in a terminal where this package is sourced. If that doesn't work, you need to set the ROS_PACKAGE_PATH to include this workspace's install folder. This can be done by running the following command in the terminal where you launch Foxglove Studio:
```bash
export ROS_PACKAGE_PATH=/absolute/path/to/this/workspace/install
```
or by adding it to your .bashrc file:
```bash
echo "export ROS_PACKAGE_PATH=/absolute/path/to/this/workspace/install" >> ~/.bashrc
```
This will make the installed packages in this workspace available to all terminals.

If that doesn't work either, you can add the path to the ROS_PACKAGE_PATH in the Foxglove Studio settings. (Click your profile icon in the top right corner, then click settings)

## Freya URDF

The Freya URDF was generated from the a CAD file using Solidworks.
These tutorials were useful:
- [URDF ROS2 Tutorial](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html)
- [Conventions for URDF files](http://wiki.ros.org/Industrial/Tutorials/Create%20a%20URDF%20for%20an%20Industrial%20Robot)
- [Export URDF from SolidWorks](http://wiki.ros.org/sw_urdf_exporter/Tutorials/Export%20an%20Assembly)
