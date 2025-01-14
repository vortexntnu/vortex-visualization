from setuptools import setup

package_name = "auv_gui"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", 
                ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="sondre",
    maintainer_email="sondre95556888@gmail.com",
    description="AUV GUI for ROS2",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "auv-gui = auv_gui.auv_gui:main",
        ],
    },
)
