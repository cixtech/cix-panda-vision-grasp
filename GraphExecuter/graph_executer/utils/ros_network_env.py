"""Align GraphExecuter ROS 2 DDS with panda_moveit_config gazebo.launch (localhost only)."""

from __future__ import annotations

import os
import subprocess
from pathlib import Path


def _fastdds_xml_candidates() -> list[Path]:
    here = Path(__file__).resolve()
    graph_executer = here.parents[1]
    repo_root = graph_executer.parents[1]
    candidates = [
        repo_root
        / 'ros2_project/install/panda_moveit_config/share/panda_moveit_config/config/fastdds_shm_only.xml',
        repo_root
        / 'ros2_project/src/panda_moveit_config/config/fastdds_shm_only.xml',
    ]
    try:
        prefix = subprocess.run(
            ['ros2', 'pkg', 'prefix', 'panda_moveit_config'],
            capture_output=True,
            text=True,
            check=False,
            timeout=3,
        )
        if prefix.returncode == 0 and prefix.stdout.strip():
            candidates.insert(
                0,
                Path(prefix.stdout.strip())
                / 'share/panda_moveit_config/config/fastdds_shm_only.xml',
            )
    except (FileNotFoundError, subprocess.TimeoutExpired, OSError):
        pass
    return candidates


def find_fastdds_shm_only_xml() -> str | None:
    for path in _fastdds_xml_candidates():
        if path.is_file():
            return str(path.resolve())
    return None


def ensure_localhost_ros_dds() -> str | None:
    """
    Match gazebo.launch DDS settings so GraphExecuter can subscribe to /image_raw.

    Skips when FASTRTPS_DEFAULT_PROFILES_FILE is already set (user override).
    """
    if os.environ.get('FASTRTPS_DEFAULT_PROFILES_FILE'):
        return os.environ['FASTRTPS_DEFAULT_PROFILES_FILE']

    xml = find_fastdds_shm_only_xml()
    if not xml:
        return None

    os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = xml
    os.environ['FASTDDS_DEFAULT_PROFILES_FILE'] = xml
    os.environ.setdefault('ROS_LOCALHOST_ONLY', '1')
    return xml
