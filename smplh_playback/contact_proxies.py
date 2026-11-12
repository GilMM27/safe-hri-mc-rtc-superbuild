"""Subject-fitted rigid contact proxies; joint attachment is encoded in body names."""
import xml.etree.ElementTree as ET
import numpy as np

# Region, attachment joint, capsule endpoint (None uses the region's principal axis).
SEGMENTS = [('torso', 0, 12), ('head', 15, None),
            ('left_upper_arm', 16, 18), ('left_forearm', 18, 20), ('left_hand', 20, None),
            ('right_upper_arm', 17, 19), ('right_forearm', 19, 21), ('right_hand', 21, None),
            ('left_thigh', 1, 4), ('left_lower_leg', 4, 7), ('left_foot', 7, None),
            ('right_thigh', 2, 5), ('right_lower_leg', 5, 8), ('right_foot', 8, None)]


def capsule_axis(points):
    centered = points - points.mean(axis=0)
    _, directions = np.linalg.eigh(centered.T @ centered)
    axis = directions[:, -1]
    if axis[np.argmax(np.abs(axis))] < 0:
        axis = -axis
    return axis


def add_contact_proxies(world, vertices, joints, weights, definitions, root):
    """Fit in the unposed, shaped SMPL frame, whose joint axes are identity."""
    scores = np.stack([weights[:, ids].sum(axis=1) for ids in definitions.values()], axis=1)
    labels = scores.argmax(axis=1)
    names = list(definitions)
    for name, joint, end in SEGMENTS:
        # Physical regions remain standard even when distance regions are customized.
        points = vertices[labels == names.index(name)] - joints[joint]
        if not len(points):
            raise ValueError(f'Cannot fit contact proxy for {name}: no skin vertices')
        target = ET.SubElement(world, 'body', name=f'smplh_contact_target_{joint}', mocap='true')
        body = ET.SubElement(world, 'body', name=f'smplh_contact_{joint}')
        ET.SubElement(body, 'freejoint', name=f'smplh_contact_free_{joint}')
        attrs = dict(name=f'smplh_contact_geom_{joint}', contype='0', conaffinity='0',
                     group='4', rgba='.9 .4 .1 .35', density='1000',
                     friction='1 .005 .0001', solref='.02 1', solimp='.9 .95 .001')
        axis = joints[end] - joints[joint] if end is not None else capsule_axis(points)
        length = np.linalg.norm(axis)
        if length < 1e-10:
            raise ValueError(f'Cannot fit zero-length contact segment: {name}')
        axis = axis / length
        along = points @ axis
        start, stop = axis * along.min(), axis * along.max()
        radius = max(.015, float(np.linalg.norm(points - along[:, None] * axis, axis=1).max()))
        attrs.update(type='capsule', size=str(radius), fromto=' '.join(map(str, np.r_[start, stop])))
        ET.SubElement(body, 'geom', **attrs)
        ET.SubElement(target, 'geom', **dict(attrs, name=f'smplh_contact_kinematic_geom_{joint}'))
    equality = ET.SubElement(root, 'equality')
    for _, joint, _ in SEGMENTS:
        ET.SubElement(equality, 'weld', name=f'smplh_contact_weld_{joint}',
                      body1=f'smplh_contact_target_{joint}', body2=f'smplh_contact_{joint}',
                      solref='.01 1', solimp='.95 .99 .001')
