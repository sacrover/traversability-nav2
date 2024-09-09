from setuptools import find_packages, setup
import os
from glob import glob

package_name = "traversability"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="sv",
    maintainer_email="mail.sacrover@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            f"pcl_to_heightmap = {package_name}.pcl_to_heightmap:main",
            f"grid_transform = {package_name}.grid_transform:main",
            f"fused_traversability_map = {package_name}.fused_traversability_map:main",
            f"terrain_surface = {package_name}.terrain_surface:main",
        ],
    },
)
