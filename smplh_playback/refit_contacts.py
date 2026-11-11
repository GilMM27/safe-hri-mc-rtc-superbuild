"""Fit existing SMPL-H contact proxies over every cached animation frame."""
import argparse
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

from .contact_proxies import SEGMENTS, capsule_axis


def refit(cache):
    cache = Path(cache)
    xml = cache / 'human.xml'
    tree = ET.parse(xml)
    vertices = np.load(cache / 'vertices.npy', mmap_mode='r')
    joints = np.load(cache / 'joints.npy', mmap_mode='r')
    poses = np.load(cache / 'qpos.npy', mmap_mode='r')
    faces = np.load(cache / 'faces.npy', mmap_mode='r')
    if poses.shape != (len(vertices), 211) or joints.shape != (len(vertices), 52, 3):
        raise ValueError('Contact fitting requires aligned SMPL-H poses, joints and vertices')
    # Cache quaternions use wxyz; SciPy accepts xyzw. The root is already in
    # world coordinates, while the remaining 51 rotations are parent-relative.
    quaternion = np.empty((len(poses), 52, 4), dtype=np.float64)
    quaternion[:, 0] = poses[:, 3:7]
    quaternion[:, 1:] = poses[:, 7:].reshape(-1, 51, 4)
    local = Rotation.from_quat(quaternion[:, :, [1, 2, 3, 0]].reshape(-1, 4)).as_matrix()
    local = local.reshape(len(poses), 52, 3, 3)
    parents = np.full(52, -1, dtype=np.int32)
    bodies = tree.getroot().find('worldbody')
    for i in range(1, 52):
        child = bodies.find(f'.//body[@name="smplh_body_{i}"]')
        if child is None:
            raise ValueError(f'Missing SMPL-H joint body {i}')
        # ElementTree has no parent lookup; find the enclosing SMPL-H body.
        for parent in bodies.iter('body'):
            if child in list(parent):
                parents[i] = int(parent.get('name').removeprefix('smplh_body_'))
                break
        if parents[i] < 0:
            raise ValueError(f'Missing SMPL-H parent for body {i}')
    world = np.empty_like(local)
    world[:, 0] = local[:, 0]
    for i in range(1, 52):
        world[:, i] = world[:, parents[i]] @ local[:, i]
    with np.load(cache / 'regions.npz') as archive:
        for name, joint, end in SEGMENTS:
            geom = bodies.find(f'./body[@name="smplh_contact_{joint}"]/geom')
            if geom is None or name not in archive:
                raise ValueError(f'Missing contact proxy or region: {name}; regenerate the cache')
            geom.attrib.pop('margin', None)
            ids = np.unique(faces[archive[name]].ravel())
            radius = 0.0
            if geom.get('fromto') is not None:
                raw = np.fromstring(geom.get('fromto'), sep=' ').reshape(2, 3)
                axis = raw[1] - raw[0]
            else:
                # Upgrade older head, hand and foot boxes without recomputing
                # the SMPL-H cache. Use the first frame's local long axis.
                first = np.asarray(vertices[0, ids], dtype=np.float64) - joints[0, joint]
                axis = capsule_axis(first @ world[0, joint])
            axis /= np.linalg.norm(axis)
            minimum, maximum = np.inf, -np.inf
            for start in range(0, len(poses), 32):
                stop = min(start + 32, len(poses))
                delta = np.asarray(vertices[start:stop, ids], dtype=np.float64) - joints[start:stop, joint, None]
                points = np.einsum('fvi,fij->fvj', delta, world[start:stop, joint])
                flat = points.reshape(-1, 3)
                along = flat @ axis
                minimum = min(minimum, float(along.min()))
                maximum = max(maximum, float(along.max()))
                radius = max(radius, float(np.linalg.norm(flat - along[:, None] * axis, axis=1).max()))
            start_point = axis * minimum
            end_point = axis * maximum
            geom.attrib.pop('pos', None)
            geom.set('type', 'capsule')
            geom.set('fromto', ' '.join(map(str, np.r_[start_point, end_point])))
            geom.set('size', str(radius + .005))
    tree.write(xml)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('cache', type=Path)
    args = parser.parse_args()
    refit(args.cache)
    print(f'Refitted contact proxies in {args.cache / "human.xml"}')


if __name__ == '__main__':
    main()
