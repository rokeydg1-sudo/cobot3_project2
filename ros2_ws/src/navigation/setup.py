from glob import glob
from setuptools import setup


package_name = "navigation"

setup(
    name=package_name,
    version="0.0.0",
    packages=[],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/config", glob("config/*")),
        ("share/" + package_name + "/maps", glob("maps/*.*")),
        ("share/" + package_name + "/maps/factory", glob("maps/factory/*.*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
)
