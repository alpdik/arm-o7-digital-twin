from glob import glob
from setuptools import find_packages, setup


package_name = "arm_o7_glove"


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
    description="5DT glove teleoperation of the O7 hand, mirrored to a real Ti5 hand.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "glove_5dt = arm_o7_glove.glove_node:main",
            "teleop_panel = arm_o7_glove.teleop_panel:main",
            "ti5_bridge = arm_o7_glove.ti5_bridge:main",
        ],
    },
)
