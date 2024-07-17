from setuptools import find_packages, setup
import os
from glob import glob

package_name = "heightmap"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
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
            "heightmap_occupancy_grid = heightmap.heightmap_occupancy_grid:main",
            "heightmap_transform = heightmap.heightmap_transform:main",
        ],
    },
)
