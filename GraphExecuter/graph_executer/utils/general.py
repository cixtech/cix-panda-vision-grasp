# coding=utf-8
"""GraphExecuter node path helpers."""

import os
import sys
from pathlib import Path


def find_nodes_folder(start_path):
    """Walk up from path to find the nodes directory; return (path, package identifier)."""
    flag_found = []
    current_path = Path(start_path).absolute()

    if current_path.is_file():
        file_name = os.path.basename(os.path.splitext(current_path)[0])
        flag_found.append(file_name)
        current_path = current_path.parent

    while True:
        nodes_path = current_path / 'nodes'
        if nodes_path.exists() and nodes_path.is_dir():
            flag_found.append('nodes')
            return str(nodes_path), ".".join(flag_found[::-1])

        if current_path.name == 'nodes':
            flag_found.append('nodes')
            return str(current_path), ".".join(flag_found[::-1])

        if current_path.parent == current_path:
            flag_found.append('nodes')
            return None, ".".join(flag_found[::-1])

        flag_found.append(os.path.basename(current_path))
        current_path = current_path.parent


def find_dir_path(dir_name):
    """Find a sys.path entry whose path segments contain dir_name."""
    for path in sys.path:
        if dir_name in path:
            return path
    return None
