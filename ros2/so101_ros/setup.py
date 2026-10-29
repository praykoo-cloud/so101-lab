from glob import glob

from setuptools import setup

package_name = "so101_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
        ("share/" + package_name + "/meshes", glob("../../model/assets/*.stl")),   # RViz 용 (package://so101_ros/meshes/)
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="max",
    maintainer_email="th2102da@gmail.com",
    description="SO-ARM101 nodes: arm, camera, color, task",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "arm_node = so101_ros.arm_node:main",
            "camera_node = so101_ros.camera_node:main",
            "color_node = so101_ros.color_node:main",
            "task_node = so101_ros.task_node:main",
        ],
    },
)
