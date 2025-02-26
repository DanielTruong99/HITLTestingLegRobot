from setuptools import find_packages, setup

package_name = "rl_controller"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            ["rl_controller/launch/robot_controller.launch.py"],
        ),
        (
            "share/" + package_name + "/configs",
            ["rl_controller/configs/leg_robot.yaml"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="humanoid",
    maintainer_email="datthanh01237344449@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "controller_node = rl_controller.controller_node:main",
            # "remote_controller_node = rl_controller.remote_controller_node:main",
        ],
    },
)
