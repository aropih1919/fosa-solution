import os
from glob import glob

from setuptools import setup

package_name = "caytu_config"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"),
         glob("config/*.yaml") + glob("config/*.xml") + glob("config/*.json")),
        (os.path.join("share", package_name, "config", "profils"),
         glob("config/profils/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Équipe Fosa",
    maintainer_email="johny@fosa.mg",
    description="Configuration unique de la solution Fosa (PARC 2026).",
    license="MIT",
    tests_require=["pytest"],
    entry_points={"console_scripts": []},
)
