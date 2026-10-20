"""CPU affinity helpers for ROS 2 launch files (taskset)."""

from __future__ import annotations

_DISABLED = frozenset({'', 'none', 'off', 'false', '-1'})


def taskset_prefix(cpu_cores: str) -> list[str]:
    """Return a ``prefix`` list for ``launch_ros.actions.Node``.

    ROS 2 launch joins list items with ``-`` for the process name when multiple
    strings are given, so use a single shell-style command string (same as gdb).
    """
    cores = (cpu_cores or '').strip()
    if cores.lower() in _DISABLED:
        return []
    return [f'taskset -c {cores}']


def merge_execute_prefix(cpu_cores: str, existing_prefix: str | None = None) -> str | None:
    """Return a shell ``prefix`` string for ``launch.actions.ExecuteProcess``."""
    cores = (cpu_cores or '').strip()
    parts: list[str] = []
    if cores.lower() not in _DISABLED:
        parts.append(f'taskset -c {cores}')
    if existing_prefix:
        parts.append(existing_prefix)
    return ' '.join(parts) if parts else None
