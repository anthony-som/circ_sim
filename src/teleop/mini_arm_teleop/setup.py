from setuptools import setup, find_packages

package_name = "mini_arm_teleop"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ibrahim",
    maintainer_email="ibrahim@todo.todo",
    description="Teleop nodes for mini arm (keyboard + PS5 + IK)",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "keyboard_teleop = mini_arm_teleop.keyboard_teleop:main",
            "keyboard_cartesian_ik_teleop = mini_arm_teleop.keyboard_cartesian_ik_teleop:main",
            "ps5_arm_teleop = mini_arm_teleop.ps5_arm_teleop:main",
            "ps5_cartesian_ik_teleop = mini_arm_teleop.ps5_cartesian_ik_teleop:main",
            "logitech3d_arm_teleop = mini_arm_teleop.logitech3d_arm_teleop:main",
        ],
    },
)

