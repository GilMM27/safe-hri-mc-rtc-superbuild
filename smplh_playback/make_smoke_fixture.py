"""Create a dependency-free synthetic cache for testing the MuJoCo bridge.

This is intentionally not an SMPL-H body or AMASS motion. It only verifies
scene merging, deterministic playback, rendering updates and JSONL output.
"""
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from .prepare import REGIONS


def write_human_xml(output, vertices, faces):
    root = ET.Element('mujoco', model='smplh_smoke')
    ET.SubElement(root, 'compiler', angle='radian')
    asset = ET.SubElement(root, 'asset')
    ET.SubElement(asset, 'mesh', name='smplh_surface', file=str((output / 'surface.obj').resolve()))
    with (output / 'surface.obj').open('w') as stream:
        for vertex in vertices[0]:
            stream.write('v ' + ' '.join(map(str, vertex)) + '\n')
        for _ in vertices[0]:
            stream.write('vn 0 0 1\n')
        for face in faces:
            stream.write('f ' + ' '.join(f'{index}//{index}' for index in face + 1) + '\n')
    world = ET.SubElement(root, 'worldbody')
    visual = ET.SubElement(world, 'body', name='smplh_visual')
    ET.SubElement(visual, 'geom', name='smplh_surface', type='mesh', mesh='smplh_surface',
                  contype='0', conaffinity='0', group='2', rgba='.65 .75 .85 1')
    bodies = []
    for index in range(52):
        parent = world if index == 0 else bodies[0]
        body = ET.SubElement(parent, 'body', name=f'smplh_body_{index}',
                             pos=('0 0 1' if index == 0 else f'{.03 * index} 0 0'))
        if index == 0:
            ET.SubElement(body, 'freejoint', name='smplh_root')
        else:
            ET.SubElement(body, 'joint', name=f'smplh_joint_{index}', type='ball', damping='0')
        ET.SubElement(body, 'inertial', pos='0 0 0', mass='.01', diaginertia='.0001 .0001 .0001')
        ET.SubElement(body, 'geom', type='sphere', size='.025', contype='0', conaffinity='0', group='3')
        bodies.append(body)
    from .contact_proxies import add_contact_proxies
    rest_joints = np.array([[0, 0, 1]] + [[.03*i, 0, 1] for i in range(1, 52)])
    add_contact_proxies(world, vertices[0], rest_joints, np.repeat(np.eye(52), 4, axis=0), REGIONS)
    ET.ElementTree(root).write(output / 'human.xml')
    (output / 'smplh.yaml').write_text(f'xmlModelPath: {json.dumps(str((output / "human.xml").resolve()))}\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--fps', type=float, default=30)
    parser.add_argument('--frames', type=int, default=120)
    args = parser.parse_args()
    if args.fps <= 0 or args.frames < 2:
        raise ValueError('FPS must be positive and at least two frames are required')
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    tetrahedron = np.array([[0, 0, 0], [.03, 0, 0], [0, .03, 0], [0, 0, .03]], dtype=np.float32)
    centers = np.array([[.03 * index, .05 * (index % 4), 1 + .08 * (index % 8)] for index in range(52)])
    rest = (centers[:, None] + tetrahedron).reshape(-1, 3)
    faces = np.concatenate([np.array([[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]]) + 4 * index
                            for index in range(52)])
    vertices = np.stack([rest + [frame * .004, 0, 0] for frame in range(args.frames)]).astype(np.float32)
    qpos = np.zeros((args.frames, 211))
    qpos[:, 0] = np.arange(args.frames) * .004
    qpos[:, 2] = 1
    qpos[:, 3] = 1
    qpos[:, 7::4] = 1
    region_faces = {name: np.arange(4 * index, 4 * index + 4)
                    for index, name in enumerate(REGIONS)}
    np.save(output / 'vertices.npy', vertices)
    np.save(output / 'faces.npy', faces)
    np.save(output / 'qpos.npy', qpos)
    np.save(output / 'joints.npy', np.repeat(centers[None], args.frames, axis=0))
    np.savez(output / 'regions.npz', **region_faces)
    (output / 'metadata.json').write_text(json.dumps({
        'cache_format': 1, 'fps': args.fps, 'frames': args.frames, 'vertices': len(vertices[0]),
        'faces': len(faces), 'region_faces': {name: len(ids) for name, ids in region_faces.items()},
        'regions': list(region_faces), 'source': 'synthetic smoke fixture',
        'joint_names': ['smplh_root'] + [f'smplh_joint_{index}' for index in range(1, 52)],
    }, indent=2))
    write_human_xml(output, vertices, faces)
    from .native_cache import write_native_cache
    write_native_cache(output)
    print(f'Prepared synthetic smoke fixture in {output}')


if __name__ == '__main__':
    main()
