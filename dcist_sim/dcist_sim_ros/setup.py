from setuptools import find_packages, setup

package_name = "dcist_sim_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Harel Biggie",
    maintainer_email="harelb@mit.edu",
    description=(
        "Sim-backed implementation of the Spot interface for spot_executor, "
        "wired to Isaac Sim over ROS2"
    ),
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
