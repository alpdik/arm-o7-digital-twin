from glob import glob
from setuptools import find_packages, setup


package_name = "arm_o7_twin"


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
    description=(
        "Fail-closed ROS 2 command arbiter for an ARM1.5 and Linker Hand O7 "
        "digital twin."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "arbiter = arm_o7_twin.node:main",
            "qualification = arm_o7_twin.qualification:main",
        ],
    },
)
