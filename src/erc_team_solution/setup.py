"""Setuptools configuration for the ERC team solution package."""

import os
from glob import glob

from setuptools import find_packages
from setuptools import setup


package_name = "erc_team_solution"


setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            os.path.join("share", package_name),
            ["package.xml"],
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="ERC 2026 Team",
    maintainer_email="team@example.com",
    description=(
        "Autonomous library-assistant solution for "
        "Emirates Robotics Competition 2026."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "orchestrator = erc_team_solution.orchestrator:main",
        ],
    },
)
