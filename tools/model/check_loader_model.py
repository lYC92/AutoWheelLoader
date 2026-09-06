"""Validate expanded mesh and primitive models against the same physical contract."""
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ros'))
from localization_geometry import visual_bounds


def main():
    mesh, primitive = map(ET.parse, sys.argv[1:3])
    for tag in ('joint', 'link'):
        a = {e.get('name'): e for e in mesh.getroot().findall(tag)}
        b = {e.get('name'): e for e in primitive.getroot().findall(tag)}
        assert a.keys() == b.keys(), f'{tag} topology changed'
        for name in a:
            if tag == 'joint':
                assert ET.tostring(a[name]) == ET.tostring(b[name]), name
            else:
                for physical in ('inertial', 'collision'):
                    assert [ET.tostring(e) for e in a[name].findall(physical)] == [ET.tostring(e) for e in b[name].findall(physical)], name
    bounds = visual_bounds(sys.argv[1])
    for name in ['front_left_wheel', 'front_right_wheel', 'rear_left_wheel', 'rear_right_wheel']:
        parts = [b for b in bounds if b[0] == name]
        low = np.min([b[2] for b in parts], axis=0)
        high = np.max([b[3] for b in parts], axis=0)
        np.testing.assert_allclose(high-low, [1.5, .55, 1.5], atol=1e-5)
        np.testing.assert_allclose(high+low, 0, atol=1e-5)
    roof = max(b[3][2] for b in bounds if b[0] == 'rear_frame')
    lidar_z = float(mesh.find("joint[@name='lidar_mount_joint']/origin").get('xyz').split()[2])
    assert lidar_z - .08 > roof + .05, 'lidar intersects cab roof'
    assert len(bounds) == 15, 'unexpected visual count'
    print('PASS  both visual modes share physics; four centered tires; lidar clears roof; all meshes readable')


if __name__ == '__main__':
    main()
