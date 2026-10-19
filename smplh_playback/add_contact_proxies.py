"""Create an opt-in MuJoCo contact model sampled from SMPL-H surface vertices.

Each small mocap sphere follows a material surface vertex. The visual mesh and
the independent SCH/FCL distance geometry are unchanged. Sparse sampling can
miss contacts between spheres; the geometry is a diagnostic approximation.
"""
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


def farthest_samples(points: np.ndarray, count: int) -> np.ndarray:
    """Deterministic coverage of one region in the cache's first frame."""
    count = min(count, len(points))
    chosen = np.empty(count, dtype=np.int64)
    center = points.mean(axis=0)
    chosen[0] = np.argmin(np.sum((points - center) ** 2, axis=1))
    nearest_sq = np.full(len(points), np.inf)
    for i in range(1, count):
        delta = points - points[chosen[i - 1]]
        nearest_sq = np.minimum(nearest_sq, np.einsum('ij,ij->i', delta, delta))
        chosen[i] = np.argmax(nearest_sq)
    return chosen


def create(cache: Path, points_per_region: int = 32, radius: float = .02):
    if points_per_region < 1 or not np.isfinite(radius) or radius <= 0:
        raise ValueError('points-per-region and radius must be positive')
    tree = ET.parse(cache / 'human.xml')
    root = tree.getroot()
    world = root.find('./worldbody')
    if world is None:
        raise ValueError('human.xml has no worldbody')
    vertices = np.load(cache / 'vertices.npy', mmap_mode='r')
    faces = np.load(cache / 'faces.npy', mmap_mode='r')
    if vertices.ndim != 3 or vertices.shape[2] != 3 or faces.ndim != 2 or faces.shape[1] != 3:
        raise ValueError('invalid SMPL-H cache geometry')
    with np.load(cache / 'regions.npz') as regions:
        for region in regions.files:
            ids = np.unique(faces[regions[region]].ravel())
            if not len(ids) or ids.min() < 0 or ids.max() >= vertices.shape[1]:
                raise ValueError(f'invalid region vertices: {region}')
            local = farthest_samples(np.asarray(vertices[0, ids], dtype=float), points_per_region)
            for vertex in ids[local]:
                position = vertices[0, vertex]
                if not np.isfinite(position).all():
                    raise ValueError(f'nonfinite region vertex: {region}')
                name = f'smplh_contact_{region}_v{vertex}'
                body = ET.SubElement(world, 'body', name=name + '_body', mocap='true',
                                     pos=' '.join(map(str, position)))
                # Zero affinity prevents proxy-proxy contacts. Robot collision
                # geoms have the default compatible affinity bit 1.
                ET.SubElement(body, 'geom', name=name, type='sphere', size=str(radius),
                              contype='1', conaffinity='0', condim='1', group='3', rgba='0 0 0 0')
    output = cache / 'human_contact.xml'
    tree.write(output)
    config = cache / 'smplh_contact.yaml'
    config.write_text('xmlModelPath: ' + json.dumps(str(output.resolve())) + '\n')
    return output, config


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('cache', type=Path)
    parser.add_argument('--points-per-region', type=int, default=32)
    parser.add_argument('--radius', type=float, default=.02, help='contact sphere radius in metres')
    args = parser.parse_args()
    output, config = create(args.cache, args.points_per_region, args.radius)
    print(f'Wrote {output} and {config}')


if __name__ == '__main__':
    main()
