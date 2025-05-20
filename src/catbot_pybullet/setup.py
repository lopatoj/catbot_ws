import os

from glob import glob
from setuptools import find_packages, setup

package_name = "catbot_pybullet"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/meshes",
            glob(os.path.join("meshes", "*")),
        ),
        (
            "share/" + package_name + "/description",
            glob(os.path.join("description", "robot.urdf")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="lopatoj",
    maintainer_email="justin@lopato.org",
    description="ROS2 node for simulating catbot with PyBullet.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "pybullet_node = catbot_pybullet.pybullet_node:main"
        ],
    },
)
