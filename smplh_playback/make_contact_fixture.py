"""Copy a native SMPL-H cache and shift its surface for a contact test.

This changes the visible/distance surface only. The cached skeleton qpos is
unchanged, so the result is a geometry fixture, not a faithful mocap scene.
"""

import argparse
from pathlib import Path
import shutil
import struct

import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path, help='source smplh_cache.bin')
    parser.add_argument('destination', type=Path, help='new smplh_cache.bin')
    parser.add_argument('--translation', type=float, nargs=3, default=(0, -0.7, 0),
                        metavar=('X', 'Y', 'Z'))
    args = parser.parse_args()
    if args.source.resolve() == args.destination.resolve():
        parser.error('source and destination must differ')
    args.destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(args.source, args.destination)
    header = struct.Struct('<8sIIIII d')
    with args.destination.open('rb') as stream:
        magic, frames, vertices, _faces, regions, nq, _fps = header.unpack(stream.read(header.size))
        if magic != b'SMPLHC1\0' or nq != 211:
            parser.error('unsupported native cache')
        for _ in range(regions):
            count = struct.unpack_from('<I', stream.read(68), 64)[0]
            stream.seek(4 * count, 1)
        offset = stream.tell() + 4 * frames * nq
    surface = np.memmap(args.destination, mode='r+', dtype='<f4', offset=offset,
                        shape=(frames, vertices, 3))
    for axis, displacement in enumerate(args.translation):
        surface[:, :, axis] += displacement
    surface.flush()
    print(f'Wrote {args.destination} with surface translation {args.translation} m')


if __name__ == '__main__':
    main()
