"""Run with SMPLH_CONTACT_TEST_BINARY pointing to compiled test_smplh_contacts.cpp."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

from smplh_playback.native_cache import write_native_cache


@unittest.skipUnless(os.getenv('SMPLH_CONTACT_TEST_BINARY'), 'Set SMPLH_CONTACT_TEST_BINARY to run native contacts')
class ContactTests(unittest.TestCase):
    def test_physical_response_and_modes(self):
        with tempfile.TemporaryDirectory() as tmp:
            cache = Path(tmp)
            subprocess.run([sys.executable, '-m', 'smplh_playback.make_smoke_fixture',
                            '--output', tmp, '--frames', '3'], check=True, capture_output=True)
            tree = ET.parse(cache / 'human.xml')
            root = tree.getroot()
            proxies = root.findall('./worldbody/body[@mocap="true"]/geom')
            self.assertEqual(len(proxies), 14)
            self.assertTrue(all('margin' not in geom.attrib for geom in proxies))
            ET.SubElement(root, 'option', gravity='0 0 0', timestep='.001')
            world = root.find('worldbody')
            body = ET.SubElement(world, 'body', name='kinova_probe', pos='0 0 3')
            ET.SubElement(body, 'freejoint', name='probe_root')
            ET.SubElement(body, 'geom', type='sphere', size='.03', mass='1', group='3')
            ET.SubElement(body, 'geom', name='robot_visual', type='sphere', size='.03',
                          contype='0', conaffinity='0', group='2')
            # Environment objects must not acquire human contact affinity.
            ET.SubElement(world, 'geom', name='environment', type='sphere', size='.03', pos='0 0 1')
            scene = cache / 'scene.xml'
            tree.write(scene)
            env = dict(os.environ, SMPLH_CACHE=tmp)
            for key in ('SMPLH_CONTACT_MODE', 'SMPLH_FREEZE_FRAME', 'SMPLH_LOOP'):
                env.pop(key, None)
            binary = env['SMPLH_CONTACT_TEST_BINARY']
            csv = cache / 'contacts.csv'
            subprocess.run([binary, str(scene)], env=dict(env, SMPLH_CONTACT_CSV=str(csv)), check=True)
            rows = csv.read_text().splitlines()
            self.assertEqual(rows[0], 'time_s,contact_index,region,robot_geom,normal_force_N,depth_m')
            self.assertGreater(len(rows), 1)
            subprocess.run([binary, str(scene)], env=dict(env, SMPLH_TEST_CONTACT_CONFIG='1'), check=True)
            subprocess.run([binary, str(scene)], env=dict(env, SMPLH_CONTACT_MODE='kinematic'), check=True)
            subprocess.run([binary, str(scene), 'impact'], env=env, check=True)
            for elliptic in (False, True):
                for sparse in (False, True):
                    estimate_env = dict(env)
                    if elliptic:
                        estimate_env['SMPLH_TEST_ELLIPTIC'] = '1'
                    if sparse:
                        estimate_env['SMPLH_TEST_SPARSE'] = '1'
                    subprocess.run([binary, str(scene), 'estimate'], env=estimate_env, check=True)
            subprocess.run([binary, str(scene)], env=dict(env, SMPLH_CONTACT_MODE='off'), check=True)
            subprocess.run([binary, str(scene)], env=dict(env, SMPLH_FREEZE_FRAME='1'), check=True)
            subprocess.run([binary, str(scene)], env=dict(env, SMPLH_LOOP='1'), check=True)
            result = subprocess.run([binary, str(scene)], env=dict(env, SMPLH_CONTACT_MODE='invalid'), capture_output=True, text=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('must be dynamic, kinematic or off', result.stderr)
            for fps in (15, 60):
                metadata = json.loads((cache / 'metadata.json').read_text())
                metadata['fps'] = fps
                (cache / 'metadata.json').write_text(json.dumps(metadata))
                write_native_cache(cache)
                subprocess.run([binary, str(scene)], env=dict(env, SMPLH_TEST_FPS=str(fps)), check=True)
            guard_tree = ET.parse(scene)
            guard_root = guard_tree.getroot()
            guard_world = guard_root.find('worldbody')
            guard_world.remove(guard_world.find('./body[@name="kinova_probe"]'))
            arm = ET.SubElement(guard_world, 'body', name='kinova_arm', pos='0 0 2')
            ET.SubElement(arm, 'joint', name='arm', type='hinge', axis='0 0 1')
            ET.SubElement(arm, 'geom', name='kinova_tip', type='sphere', pos='.3 0 0',
                          size='.03', mass='1', group='3')
            actuator = ET.SubElement(guard_root, 'actuator')
            ET.SubElement(actuator, 'position', name='arm_servo', joint='arm', kp='2000', kv='100',
                          forcerange='-105 105')
            for body in guard_world.findall('body'):
                if not body.get('name', '').startswith('smplh_contact_'):
                    continue
                geom = body.find('geom')
                geom.set('type', 'capsule')
                geom.set('size', '.06' if body.get('name') == 'smplh_contact_0' else '.001')
                geom.set('fromto', '.3 0 0 .3001 0 0' if body.get('name') == 'smplh_contact_0'
                         else '10 10 10 10.0001 10 10')
            guard_scene = cache / 'guard.xml'
            guard_tree.write(guard_scene)
            subprocess.run([binary, str(guard_scene), 'guard'], env=env, check=True)
            # Emulate an old prepared cache without contact metadata.
            for body in list(world.findall('body')):
                if body.get('mocap') == 'true':
                    world.remove(body)
            root.remove(root.find('equality'))
            tree.write(scene)
            result = subprocess.run([binary, str(scene)], env=env, capture_output=True, text=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('regenerate the cache', result.stderr)
            self.assertIn('SMPLH_CONTACT_MODE=off', result.stderr)


if __name__ == '__main__':
    unittest.main()
