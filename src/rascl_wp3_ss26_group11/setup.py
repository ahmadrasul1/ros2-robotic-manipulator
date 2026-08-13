from pathlib import Path

from setuptools import find_packages, setup

package_name = "rascl_wp3_ss26_group11"


def install_tree(source: str, destination: str):
    """Preserve subdirectories when installing package data."""
    source_root = Path(source)
    groups: dict[str, list[str]] = {}
    for path in source_root.rglob("*"):
        if not path.is_file():
            continue
        relative_parent = path.parent.relative_to(source_root)
        target = str(Path(destination) / relative_parent)
        groups.setdefault(target, []).append(str(path))
    return sorted(groups.items())


data_files = [
    ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
    (f"share/{package_name}", ["package.xml"]),
]
for source in ("launch", "config", "trajectories", "scripts"):
    data_files.extend(install_tree(source, f"share/{package_name}/{source}"))

setup(
    name=package_name,
    version="0.0.3",
    packages=find_packages(exclude=["test"]),
    data_files=data_files,
    install_requires=[
        "setuptools",
        "PyYAML",
        "numpy",
        "roboticstoolbox-python==1.3.1",
        "spatialmath-python",
    ],
    zip_safe=False,
    maintainer="Group 11",
    maintainer_email="group11@example.invalid",
    description="RASCL WP3 offline and online minimum-jerk pick-and-place planning.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "wp3_tsk1 = rascl_wp3_ss26_group11.wp3_tsk1:main",
            "wp3_tsk2 = rascl_wp3_ss26_group11.wp3_tsk2:main",
            "publish_task2_cube = rascl_wp3_ss26_group11.task2_cube_publisher:main",
        ],
    },
)
