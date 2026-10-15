"""Synthetic, unrestricted fixtures exercise the real SMPL-H and MuJoCo tooling."""
from pathlib import Path
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
        data=mujoco.MjData(model); data.qpos[:]=self.qpos[0]
        mujoco.mj_forward(model,data)
        joints=np.load(self.cache/'joints.npy')
        for i in range(52):
            bid=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,f'smplh_body_{i}')
            np.testing.assert_allclose(data.xpos[bid],joints[0,i],atol=1e-6)
        self.assertEqual(model.nq,211)
        np.testing.assert_allclose(self.vertices[0][::4],self.rest,atol=1e-6)
        mid=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_MESH,'smplh_surface')
        self.assertEqual(model.mesh_vertnum[mid],len(self.vertices[0]))
        self.assertEqual(model.mesh_normalnum[mid],len(self.vertices[0]))

    def test_mesh_skeleton_consistency(self):
        import mujoco
        model=mujoco.MjModel.from_xml_path(str(self.cache/'human.xml')); data=mujoco.MjData(model)
        joints=np.load(self.cache/'joints.npy')
        for frame in range(3):
            data.qpos[:]=self.qpos[frame]; mujoco.mj_forward(model,data)
            for i in range(52):
                bid=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,f'smplh_body_{i}')
                np.testing.assert_allclose(data.xpos[bid],joints[frame,i],atol=1e-6)
                np.testing.assert_allclose(self.vertices[frame,4*i],joints[frame,i],atol=1e-6)

    def test_regions(self):
        self.assertEqual(set(self.regions),set(REGIONS))
        ids=np.concatenate(list(self.regions.values()))
        self.assertEqual(len(np.unique(ids)),len(self.faces))
        self.assertTrue(all(len(x)>0 for x in self.regions.values()))


if __name__=='__main__': unittest.main()
