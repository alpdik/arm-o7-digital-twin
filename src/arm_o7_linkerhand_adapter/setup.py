from glob import glob
from setuptools import find_packages, setup


package_name = "arm_o7_linkerhand_adapter"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ARM O7 Twin Maintainers",
    maintainer_email="maintainer@example.com",
    description="Fail-closed LinkerHand O7 ROS 2 SDK adapter and real-state mux.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "o7_adapter = arm_o7_linkerhand_adapter.adapter_node:main",
            "joint_state_mux = arm_o7_linkerhand_adapter.joint_state_mux:main",
        ],
    },
)
