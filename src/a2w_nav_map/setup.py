from setuptools import find_packages, setup


package_name = "a2w_nav_map"

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
    description="Offline Map Bundle V1 to Nav2 occupancy map converter.",
    license="Apache-2.0",
    entry_points={"console_scripts": ["pcd_to_occupancy = a2w_nav_map.cli:main"]},
)

