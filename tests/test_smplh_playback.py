"""Synthetic, unrestricted fixtures exercise the real SMPL-H and MuJoCo tooling."""
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
import numpy as np
from smplh_playback.prepare import REGIONS


def fixture(directory):
    # Artificial SMPL-H parameters, NOT a redistributed human model.
    n=52; nv=n*4
    parents=np.array([-1]+[0]*51)
    positions=np.array([[i*.1, (i%3)*.2, 1+(i%5)*.05] for i in range(n)],dtype=np.float32)
    tetra=np.array([[0,0,0],[.03,0,0],[0,.03,0],[0,0,.03]],dtype=np.float32)
    v=(positions[:,None]+tetra).reshape(-1,3)
    faces=np.concatenate([np.array([[0,2,1],[0,1,3],[0,3,2],[1,2,3]])+4*i for i in range(n)])
    weights=np.repeat(np.eye(n),4,axis=0).astype(np.float32)
    regressor=np.zeros((n,nv),dtype=np.float32)
    for i in range(n): regressor[i,4*i]=1
    path=directory/'synthetic.npz'
    np.savez(path,v_template=v,f=faces,shapedirs=np.zeros((nv,3,16),np.float32),
             posedirs=np.zeros((nv,3,459),np.float32),J_regressor=regressor,weights=weights,
             kintree_table=np.stack([parents,np.arange(n)]),
             hands_componentsl=np.eye(45,dtype=np.float32),hands_componentsr=np.eye(45,dtype=np.float32),
             hands_meanl=np.zeros(45,np.float32),hands_meanr=np.zeros(45,np.float32))
    poses=np.zeros((3,156),np.float32); poses[1:,3+16*3:3+16*3+3]=[0,0,.2]
    # Pose 16 is upper arm; direct pose offset is 16*3.
    poses[1:,48:51]=[0,0,.2]
    np.savez(directory/'motion.npz',poses=poses,trans=np.array([[0,0,0],[.2,0,0],[.4,0,0]],np.float32),
             betas=np.zeros(16,np.float32),gender=np.array('male'),mocap_framerate=np.array(10.))
    subprocess.run([sys.executable,'-m','smplh_playback.prepare','--model',str(path),
                    '--amass',str(directory/'motion.npz'),'--output',str(directory/'cache')],check=True)
    return directory/'cache',positions


class PipelineTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp=tempfile.TemporaryDirectory(); cls.root=Path(cls.tmp.name)
        cls.cache,cls.rest=fixture(cls.root)
        cls.vertices=np.load(cls.cache/'vertices.npy',mmap_mode='r')
        cls.qpos=np.load(cls.cache/'qpos.npy',mmap_mode='r')
        cls.faces=np.load(cls.cache/'faces.npy')
        with np.load(cls.cache/'regions.npz') as regions:
            cls.regions={name:regions[name] for name in regions.files}

    @classmethod
    def tearDownClass(cls): cls.tmp.cleanup()

    def test_load_and_frame_zero(self):
        import mujoco
        model=mujoco.MjModel.from_xml_path(str(self.cache/'human.xml'))
        data=mujoco.MjData(model); data.qpos[:211]=self.qpos[0]
        mujoco.mj_forward(model,data)
        joints=np.load(self.cache/'joints.npy')
        for i in range(52):
            bid=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,f'smplh_body_{i}')
            np.testing.assert_allclose(data.xpos[bid],joints[0,i],atol=1e-6)
        self.assertEqual(model.nq,211 + 14*7)
        np.testing.assert_allclose(self.vertices[0][::4],self.rest,atol=1e-6)
        mid=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_MESH,'smplh_surface')
        self.assertEqual(model.mesh_vertnum[mid],len(self.vertices[0]))
        self.assertEqual(model.mesh_normalnum[mid],len(self.vertices[0]))

    def test_mesh_skeleton_consistency(self):
        import mujoco
        model=mujoco.MjModel.from_xml_path(str(self.cache/'human.xml')); data=mujoco.MjData(model)
        joints=np.load(self.cache/'joints.npy')
        for frame in range(3):
            data.qpos[:211]=self.qpos[frame]; mujoco.mj_forward(model,data)
            for i in range(52):
                bid=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,f'smplh_body_{i}')
                np.testing.assert_allclose(data.xpos[bid],joints[frame,i],atol=1e-6)
                np.testing.assert_allclose(self.vertices[frame,4*i],joints[frame,i],atol=1e-6)

    def test_contact_proxies_cover_cached_surface(self):
        import mujoco
        from smplh_playback.contact_proxies import SEGMENTS
        model=mujoco.MjModel.from_xml_path(str(self.cache/'human.xml'))
        data=mujoco.MjData(model)
        for frame in range(3):
            offset=0
            for index in range(52):
                name='smplh_root' if index==0 else f'smplh_joint_{index}'
                joint=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT,name)
                size=7 if index==0 else 4
                data.qpos[model.jnt_qposadr[joint]:model.jnt_qposadr[joint]+size]=self.qpos[frame,offset:offset+size]
                offset+=size
            mujoco.mj_forward(model,data)
            for name,index,_ in SEGMENTS:
                skeleton=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,f'smplh_body_{index}')
                contact=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,f'smplh_contact_target_{index}')
                mocap=model.body_mocapid[contact]
                data.mocap_pos[mocap]=data.xpos[skeleton]
                data.mocap_quat[mocap]=data.xquat[skeleton]
                free=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT,f'smplh_contact_free_{index}')
                q=model.jnt_qposadr[free]
                data.qpos[q:q+3]=data.mocap_pos[mocap]
                data.qpos[q+3:q+7]=data.mocap_quat[mocap]
            mujoco.mj_forward(model,data)
            for name,index,_ in SEGMENTS:
                geom=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,f'smplh_contact_geom_{index}')
                ids=np.unique(self.faces[self.regions[name]].ravel())
                points=(self.vertices[frame,ids]-data.geom_xpos[geom]) @ data.geom_xmat[geom].reshape(3,3)
                self.assertEqual(model.geom_type[geom],mujoco.mjtGeom.mjGEOM_CAPSULE)
                radius,half=model.geom_size[geom,:2]
                outside=np.linalg.norm(np.c_[points[:,:2],np.maximum(np.abs(points[:,2])-half,0)],axis=1)-radius
                self.assertLessEqual(float(outside.max()),1e-4,(frame,name))

    def test_refit_upgrades_terminal_boxes_to_capsules(self):
        import xml.etree.ElementTree as ET
        from smplh_playback.refit_contacts import refit
        with tempfile.TemporaryDirectory() as tmp:
            cache=Path(tmp)/'cache'
            shutil.copytree(self.cache,cache)
            xml=cache/'human.xml'
            tree=ET.parse(xml)
            for joint in (15,20,21,7,8):
                geom=tree.getroot().find(f'./worldbody/body[@name="smplh_contact_{joint}"]/geom')
                geom.set('type','box')
                geom.set('pos','0 0 0')
                geom.set('size','.1 .1 .1')
                geom.attrib.pop('fromto')
            tree.write(xml)
            refit(cache)
            tree=ET.parse(xml)
            for joint in (15,20,21,7,8):
                geom=tree.getroot().find(f'./worldbody/body[@name="smplh_contact_{joint}"]/geom')
                self.assertEqual(geom.get('type'),'capsule')
                self.assertIsNone(geom.get('pos'))
                self.assertEqual(len(np.fromstring(geom.get('fromto'),sep=' ')),6)
            import mujoco
            mujoco.MjModel.from_xml_path(str(xml))


    def test_regions(self):
        self.assertEqual(set(self.regions),set(REGIONS))
        ids=np.concatenate(list(self.regions.values()))
        self.assertEqual(len(np.unique(ids)),len(self.faces))
        self.assertTrue(all(len(x)>0 for x in self.regions.values()))


if __name__=='__main__': unittest.main()
