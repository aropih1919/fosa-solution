import os
from glob import glob

from setuptools import setup

package_name = "caytu_config"

# Chaque carte livrée (dossier sous config/cartes) est installée à part pour
# conserver son nom dans share/caytu_config/config/cartes/<carte>.
cartes = [
    (os.path.join("share", package_name, "config", "cartes",
                  os.path.basename(dossier)), glob(os.path.join(dossier, "*")))
    for dossier in glob("config/cartes/*")
    if os.path.isdir(dossier)
]

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
    ] + cartes,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Équipe Fosa",
    maintainer_email="johny@fosa.mg",
    description="Configuration unique de la solution Fosa (PARC 2026).",
    license="MIT",
    tests_require=["pytest"],
    entry_points={"console_scripts": []},
)
