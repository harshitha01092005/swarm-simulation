from glob import glob
from setuptools import find_packages, setup

PACKAGE = "drone_swarm_simulator"

setup(
    name=PACKAGE,
    version="1.0.0",
    packages=find_packages(exclude=["tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + PACKAGE]),
        ("share/" + PACKAGE, ["package.xml"]),
        ("share/" + PACKAGE + "/launch", glob("launch/*.launch.py")),
        ("share/" + PACKAGE + "/config", glob("config/*.yaml")),
        ("share/" + PACKAGE + "/config", glob("config/*.json")),
        ("share/" + PACKAGE + "/worlds", glob("worlds/*.sdf")),
        ("share/" + PACKAGE + "/models/drone", glob("models/drone/*")),
        ("share/" + PACKAGE + "/assets", glob("assets/*.png")),
    ],
    package_data={"drone_swarm.gui": ["web/*", "web/vendor/*"]},
    install_requires=["setuptools", "numpy>=1.26,<3", "scipy>=1.11,<2", "opencv-python-headless>=4.6,<5"],
    extras_require={"desktop": ["PySide6>=6.11,<6.12"]},
    python_requires=">=3.10",
    zip_safe=True,
    maintainer="Drone Swarm Simulator contributors",
    maintainer_email="maintainers@example.com",
    description="Real-time drone swarm formation and visual pattern simulation",
    license="MIT",
    entry_points={"console_scripts": [
        "drone_demo = drone_swarm.ros_nodes.drone_demo:main",
        "communication_smoke_test = drone_swarm.ros_nodes.communication_smoke_test:main",
        "fleet_runtime = drone_swarm.ros_nodes.fleet_runtime:main",
        "swarm_desktop = drone_swarm.gui.main_window:main",
    ]},
)
