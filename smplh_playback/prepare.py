"""Bake licensed AMASS/SMPL-H input into a local, simulation-ready cache."""
import argparse
import json
import os
from pathlib import Path
import numpy as np

REGIONS = {
    'head': [15], 'torso': [0, 3, 6, 9, 12, 13, 14],
    'left_upper_arm': [16], 'left_forearm': [18], 'left_hand': [20, *range(22, 37)],
    'right_upper_arm': [17], 'right_forearm': [19], 'right_hand': [21, *range(37, 52)],
    'left_thigh': [1], 'left_lower_leg': [4], 'left_foot': [7, 10],
    'right_thigh': [2], 'right_lower_leg': [5], 'right_foot': [8, 11],
}


def segment(weights, faces, definitions=REGIONS):
    # Sum skinning influence per region; assign triangles by their mean influence.
    scores = np.stack([weights[:, ids].sum(axis=1) for ids in definitions.values()], axis=1)
    labels = scores[faces].mean(axis=1).argmax(axis=1)
    result = {name: np.flatnonzero(labels == i) for i, name in enumerate(definitions)}
    if any(len(ids) == 0 for ids in result.values()):
        raise ValueError('Every body region must contain triangles')
    return result


def main():
    import torch
    import smplx
    from smplx.lbs import batch_rodrigues, batch_rigid_transform, vertices2joints
    from scipy.spatial.transform import Rotation
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--amass', default=os.getenv('AMASS_SEQUENCE'), required=not os.getenv('AMASS_SEQUENCE'))
    p.add_argument('--model', default=os.getenv('SMPLH_MODEL'), required=not os.getenv('SMPLH_MODEL'))
    p.add_argument('--output', default=os.getenv('SMPLH_CACHE'), required=not os.getenv('SMPLH_CACHE'))
    p.add_argument('--fps', type=float, help='Playback frame rate override; changes playback speed')
    p.add_argument('--rotation', type=float, nargs=3, default=[0, 0, 0], help='World XYZ Euler degrees')
    p.add_argument('--translation', type=float, nargs=3, default=[0, 0, 0])
    p.add_argument('--regions', help='JSON mapping of region names to SMPL-H joint indices')
    args = p.parse_args()
    out = Path(args.output); out.mkdir(parents=True, exist_ok=True)
    a = np.load(args.amass, allow_pickle=False)
    poses = a['poses'].astype(np.float32)
    if poses.ndim != 2 or poses.shape[1] != 156 or not len(poses):
        raise ValueError('Expected AMASS SMPL-H poses with shape (frames,156)')
    fps = args.fps or float(a['mocap_framerate'])
    if not np.isfinite(fps) or fps <= 0: raise ValueError('FPS must be positive')
    gender = str(a['gender'].item()); gender = gender.removeprefix("b'").removesuffix("'")
    model = smplx.SMPLH(args.model, gender=gender, ext=(Path(args.model).suffix.lstrip('.') if Path(args.model).is_file() else 'pkl'),
                        use_pca=False, flat_hand_mean=True, num_betas=min(16, len(a['betas'])))
    # Only the 52 kinematic joints are required; disable optional landmark vertices.
    model.vertex_joint_selector.extra_joints_idxs = torch.empty(0, dtype=torch.long)
    beta = torch.tensor(a['betas'][:model.num_betas].astype(np.float32)[None])
    with torch.no_grad():
        shaped = model.v_template + torch.einsum('bl,vcl->bvc', beta, model.shapedirs)
        rest = vertices2joints(model.J_regressor, shaped)
    world = Rotation.from_euler('xyz', args.rotation, degrees=True).as_matrix()
    shift = np.array(args.translation)
    n = len(poses); nv = len(model.v_template)
    vertices = np.lib.format.open_memmap(out/'vertices.npy', mode='w+', dtype='float32', shape=(n,nv,3))
    qpos = np.zeros((n, 7 + 51*4))
    joints = np.zeros((n,52,3))
    for start in range(0, n, 64):
        stop = min(start+64,n); pose = torch.tensor(poses[start:stop]); count = stop-start
        trans = torch.tensor(a['trans'][start:stop].astype(np.float32))
        with torch.no_grad():
            result = model(betas=beta.expand(count,-1), global_orient=pose[:,:3], body_pose=pose[:,3:66],
                           left_hand_pose=pose[:,66:111], right_hand_pose=pose[:,111:156], transl=trans)
            rotations = batch_rodrigues(pose.reshape(-1,3)).reshape(count,52,3,3)
            posed_joints, _ = batch_rigid_transform(rotations, rest.expand(count,-1,-1), model.parents)
        vertices[start:stop] = result.vertices.numpy() @ world.T + shift
        joints[start:stop] = (posed_joints.numpy()+trans.numpy()[:,None]) @ world.T + shift
        r = rotations.numpy(); r[:,0] = world @ r[:,0]
        quat = Rotation.from_matrix(r.reshape(-1,3,3)).as_quat().reshape(count,52,4)[:,:,[3,0,1,2]]
        qpos[start:stop,:3] = joints[start:stop,0]
        qpos[start:stop,3:7] = quat[:,0]
        qpos[start:stop,7:] = quat[:,1:].reshape(count,-1)
    definitions = json.loads(Path(args.regions).read_text()) if args.regions else REGIONS
    regions = segment(model.lbs_weights.numpy(), model.faces, definitions)
    np.save(out/'faces.npy',model.faces)
    np.save(out/'qpos.npy',qpos); np.save(out/'joints.npy',joints)
    np.savez(out/'regions.npz',**regions)
    metadata = {'cache_format':1,'fps':fps,'frames':n,'vertices':nv,'faces':len(model.faces),
                'region_faces':{name:len(ids) for name,ids in regions.items()},
                'regions':list(regions),'source':Path(args.amass).name,
                'joint_names':['smplh_root']+[f'smplh_joint_{i}' for i in range(1,52)]}
    (out/'metadata.json').write_text(json.dumps(metadata,indent=2))
    # Full surface is visual-only; simplified skeleton geoms never participate in distance queries.
    import xml.etree.ElementTree as ET
    root=ET.Element('mujoco',model='smplh'); ET.SubElement(root,'compiler',angle='radian')
    asset=ET.SubElement(root,'asset'); ET.SubElement(asset,'mesh',name='smplh_surface',file=str((out/'surface.obj').resolve()))
    with (out/'surface.obj').open('w') as f:
        for v in vertices[0]: f.write('v '+' '.join(map(str,v))+'\n')
        for v in vertices[0]: f.write('vn 0 0 1\n')
        for face in model.faces: f.write('f '+' '.join(f'{i}//{i}' for i in face+1)+'\n')
    wb=ET.SubElement(root,'worldbody')
    visual=ET.SubElement(wb,'body',name='smplh_visual')
    ET.SubElement(visual,'geom',name='smplh_surface',type='mesh',mesh='smplh_surface',contype='0',conaffinity='0',group='2',rgba='.65 .75 .85 1')
    bodies=[]; rest=rest[0].numpy(); parents=model.parents.numpy()
    for i in range(52):
        parent=wb if i==0 else bodies[parents[i]]
        offset=np.zeros(3) if i==0 else rest[i]-rest[parents[i]]
        body=ET.SubElement(parent,'body',name=f'smplh_body_{i}',pos=' '.join(map(str,offset)))
        if i==0: ET.SubElement(body,'freejoint',name='smplh_root')
        else: ET.SubElement(body,'joint',name=f'smplh_joint_{i}',type='ball',damping='0')
        ET.SubElement(body,'inertial',pos='0 0 0',mass='.01',diaginertia='.0001 .0001 .0001')
        ET.SubElement(body,'geom',type='sphere',size='.025',contype='0',conaffinity='0',group='3',rgba='.9 .5 .2 0.3')
        bodies.append(body)
    ET.ElementTree(root).write(out/'human.xml')
    (out/'smplh.yaml').write_text('xmlModelPath: '+json.dumps(str((out/'human.xml').resolve()))+'\n')
    from .native_cache import write_native_cache
    write_native_cache(out)
    print(f'Prepared {n} frames at {fps} FPS in {out}')

if __name__ == '__main__': main()
