#!/usr/bin/env python3
"""Generate low-poly collision OBJ meshes (~100-200 triangles) from visual meshes."""

from __future__ import annotations

import re
import sys
from pathlib import Path

import numpy as np


def load_obj(path: Path) -> tuple[np.ndarray, list[list[int]]]:
    verts: list[list[float]] = []
    faces: list[list[int]] = []
    for line in path.read_text().splitlines():
        if line.startswith('v '):
            parts = line.split()
            verts.append([float(parts[1]), float(parts[2]), float(parts[3])])
        elif line.startswith('f '):
            idx = [int(token.split('/')[0]) - 1 for token in line.split()[1:]]
            if len(idx) == 3:
                faces.append(idx)
            elif len(idx) == 4:
                faces.extend([[idx[0], idx[1], idx[2]], [idx[0], idx[2], idx[3]]])
    return np.asarray(verts, dtype=np.float64), faces


def load_dae(path: Path, scale: float = 1.0, unit_meter: float = 0.0254) -> tuple[np.ndarray, list[list[int]]]:
    text = path.read_text()
    pos_match = re.search(
        r'<float_array id="[^"]*-mesh-positions-array" count="(\d+)">([^<]+)</float_array>',
        text,
    )
    if pos_match is None:
        raise RuntimeError(f'No positions found in {path}')
    nums = [float(x) for x in pos_match.group(2).split()]
    verts = np.asarray(nums, dtype=np.float64).reshape(-1, 3) * unit_meter * scale

    faces: list[list[int]] = []
    for block in re.finditer(
        r'<polylist\b[^>]*>.*?<vcount>([^<]+)</vcount>.*?<p>([^<]+)</p>.*?</polylist>',
        text,
        flags=re.DOTALL,
    ):
        vcounts = [int(x) for x in block.group(1).split()]
        indices = [int(x) for x in block.group(2).split()]
        cursor = 0
        for vc in vcounts:
            poly = indices[cursor:cursor + vc]
            cursor += vc
            for i in range(1, vc - 1):
                faces.append([poly[0], poly[i], poly[i + 1]])
    return verts, faces


def _cluster_faces(
    verts: np.ndarray, faces: list[list[int]], eps: float
) -> tuple[np.ndarray, list[list[int]]]:
    keys = np.round(verts / eps).astype(np.int64)
    mapping: dict[tuple[int, int, int], int] = {}
    new_verts: list[np.ndarray] = []
    new_faces: list[list[int]] = []

    for face in faces:
        remapped: list[int] = []
        valid = True
        for vi in face:
            key = tuple(keys[vi])
            if key not in mapping:
                mapping[key] = len(new_verts)
                new_verts.append(verts[vi])
            remapped.append(mapping[key])
        if remapped[0] == remapped[1] or remapped[1] == remapped[2] or remapped[0] == remapped[2]:
            continue
        new_faces.append(remapped)
    return np.asarray(new_verts, dtype=np.float64), new_faces


def _subsample_faces(
    faces: list[list[int]], target: int, low: int, high: int
) -> list[list[int]]:
    unique = _dedupe_faces(faces)
    count = len(unique)
    if count <= high:
        return unique
    pick = min(max(target, low), high, count)
    indices = np.linspace(0, count - 1, pick, dtype=int)
    return [unique[int(i)] for i in indices]


def decimate_to_target(
    verts: np.ndarray,
    faces: list[list[int]],
    target: int = 150,
    low: int = 100,
    high: int = 200,
) -> tuple[np.ndarray, list[list[int]]]:
    if not faces:
        raise RuntimeError('Mesh has no faces')

    lo_eps, hi_eps = 1e-5, max(np.ptp(verts, axis=0).max() * 0.25, 1e-3)
    best_verts, best_faces = verts, _dedupe_faces(faces)
    best_count = len(best_faces)

    for _ in range(40):
        eps = (lo_eps + hi_eps) / 2.0
        cand_verts, cand_faces = _cluster_faces(verts, faces, eps)
        count = len(_dedupe_faces(cand_faces))
        if low <= count <= high:
            return cand_verts, _dedupe_faces(cand_faces)
        if abs(count - target) < abs(best_count - target):
            best_verts, best_faces, best_count = cand_verts, _dedupe_faces(cand_faces), count
        if count > high:
            lo_eps = eps
        else:
            hi_eps = eps

    if low <= best_count <= high:
        return best_verts, best_faces

    sampled = _subsample_faces(faces, target=target, low=low, high=high)
    return verts, sampled


def _dedupe_faces(faces: list[list[int]]) -> list[list[int]]:
    seen: set[tuple[int, int, int]] = set()
    unique: list[list[int]] = []
    for a, b, c in faces:
        key = tuple(sorted((a, b, c)))
        if key in seen:
            continue
        seen.add(key)
        unique.append([a, b, c])
    return unique


def _vertex_normals(verts: np.ndarray, faces: list[list[int]]) -> np.ndarray:
    normals = np.zeros_like(verts)
    for a, b, c in faces:
        edge1 = verts[b] - verts[a]
        edge2 = verts[c] - verts[a]
        face_normal = np.cross(edge1, edge2)
        length = np.linalg.norm(face_normal)
        if length < 1e-12:
            continue
        face_normal /= length
        normals[a] += face_normal
        normals[b] += face_normal
        normals[c] += face_normal
    for i, normal in enumerate(normals):
        length = np.linalg.norm(normal)
        if length < 1e-12:
            normals[i] = np.array([0.0, 0.0, 1.0])
        else:
            normals[i] = normal / length
    return normals


def write_obj(path: Path, verts: np.ndarray, faces: list[list[int]]) -> None:
    normals = _vertex_normals(verts, faces)
    lines = ['# simplified collision mesh', f'# triangles: {len(faces)}']
    for x, y, z in verts:
        lines.append(f'v {x:.6f} {y:.6f} {z:.6f}')
    for nx, ny, nz in normals:
        lines.append(f'vn {nx:.6f} {ny:.6f} {nz:.6f}')
    for a, b, c in faces:
        ia, ib, ic = a + 1, b + 1, c + 1
        lines.append(f'f {ia}//{ia} {ib}//{ib} {ic}//{ic}')
    path.write_text('\n'.join(lines) + '\n')


def main() -> int:
    worlds = Path(__file__).resolve().parents[1] / 'worlds'
    target = 150

    cole_src = worlds / 'cole_hammer' / 'meshes' / 'model.obj'
    cole_out = worlds / 'cole_hammer' / 'meshes' / 'collision.obj'
    cole_verts, cole_faces = load_obj(cole_src)
    cole_verts += np.array([0.0006, 0.0088, -0.048])
    cole_verts, cole_faces = decimate_to_target(cole_verts, cole_faces, target=target)
    write_obj(cole_out, cole_verts, cole_faces)
    print(f'cole_hammer: {len(cole_faces)} triangles -> {cole_out}')

    hammer_src = worlds / 'hammer' / 'meshes' / 'hammer.dae'
    hammer_out = worlds / 'hammer' / 'meshes' / 'hammer_collision.obj'
    hammer_verts, hammer_faces = load_dae(hammer_src, scale=0.48)
    hammer_verts, hammer_faces = decimate_to_target(hammer_verts, hammer_faces, target=target)
    write_obj(hammer_out, hammer_verts, hammer_faces)
    print(f'hammer: {len(hammer_faces)} triangles -> {hammer_out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
