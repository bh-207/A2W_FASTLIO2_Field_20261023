from setuptools import find_packages, setup


package_name = "a2w_motion_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="AaronYanC",
    maintainer_email="AaronYanC@users.noreply.github.com",
    description="Fail-closed Twist to Unitree A2-W sport request bridge.",
    license="Apache-2.0",
    entry_points={"console_scripts": ["motion_bridge = a2w_motion_bridge.node:main",
                                     "axis_probe = a2w_motion_bridge.axis_probe:main"]},
)
