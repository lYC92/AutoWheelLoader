"""Repair the checked-in L580 visual assets; run with Blender --background.

Original STLs are retained. Generated parts go in meshes/l580/visual.
Cuts are geometric and capped, rather than discarding triangles by centroid.
"""
from pathlib import Path
import struct
import hashlib
import json

import bmesh
import numpy as np
from mathutils import Vector

ROOT = Path(__file__).resolve().parents[2] / 'ros_ws/src/loader_description/meshes/l580'
OUT = ROOT / 'visual'
DTYPE = np.dtype([('n', '<f4', (3,)), ('v', '<f4', (3, 3)), ('a', '<u2')])


def read(name):
    return np.fromfile(ROOT / name, dtype=DTYPE, offset=84)['v'].copy()


def cut(triangles, axis, location, keep_negative):
    bm = bmesh.new()
    vertices, inverse = np.unique(triangles.reshape(-1, 3), axis=0, return_inverse=True)
    verts = [bm.verts.new(v) for v in vertices]
    for indices in inverse.reshape(-1, 3):
        if len(set(indices)) == 3:
            try:
                bm.faces.new([verts[i] for i in indices])
            except ValueError:
                pass
    normal = Vector([int(i == axis) for i in range(3)])
    point = normal * location
    result = bmesh.ops.bisect_plane(bm, geom=list(bm.verts)+list(bm.edges)+list(bm.faces),
                                  dist=1e-6, plane_co=point, plane_no=normal,
                                  clear_outer=keep_negative, clear_inner=not keep_negative)
    edges = [e for e in result['geom_cut'] if isinstance(e, bmesh.types.BMEdge) and e.is_boundary]
    if edges:
        bmesh.ops.holes_fill(bm, edges=edges, sides=0)
    bmesh.ops.triangulate(bm, faces=list(bm.faces))
    bmesh.ops.recalc_face_normals(bm, faces=list(bm.faces))
    out = np.asarray([[tuple(v.co) for v in f.verts] for f in bm.faces], dtype=np.float32)
    bm.free()
    return out


def write(name, triangles):
    data = np.zeros(len(triangles), dtype=DTYPE)
    data['v'] = triangles
    normals = np.cross(triangles[:, 1]-triangles[:, 0], triangles[:, 2]-triangles[:, 0])
    data['n'] = normals / np.maximum(np.linalg.norm(normals, axis=1)[:, None], 1e-12)
    with (OUT / name).open('wb') as stream:
        stream.write(b'L580 visual repair; see ../README.md'.ljust(80, b' '))
        stream.write(struct.pack('<I', len(data)))
        stream.write(data.tobytes())
    print(name, len(data), 'triangles', np.round(np.ptp(triangles.reshape(-1, 3), axis=0), 4))


def main():
    OUT.mkdir(exist_ok=True)
    body = np.concatenate([read('rear_body.STL'), read('front_body.STL')])
    # Keep the entire cab on the rear chassis. The old x=0 centroid split cut it.
    rear = cut(body, 0, .68, True)
    front = cut(body, 0, .68, False)
    centers = rear.mean(axis=1)
    normals = np.cross(rear[:, 1]-rear[:, 0], rear[:, 2]-rear[:, 0])
    normals /= np.maximum(np.linalg.norm(normals, axis=1)[:, None], 1e-12)
    glass = ((centers[:, 0] > -1.25) & (centers[:, 0] < .58)
             & (centers[:, 2] > 1.48) & (centers[:, 2] < 2.43)
             & (abs(normals[:, 2]) < .55)
             & ((abs(centers[:, 1]) > .49) | (centers[:, 0] > .30)))
    write('rear_body.STL', rear[~glass])
    write('cab_glass.STL', rear[glass])
    write('front_body.STL', front)
    # The source half-wheel still contains triangles stretching to the far axle.
    # Cut just inside its sidewall, close the cut and center the tire on its joint.
    wheel = cut(read('wheel.STL'), 1, -.52, True)
    low, high = wheel.reshape(-1, 3).min(axis=0), wheel.reshape(-1, 3).max(axis=0)
    wheel -= (low + high) / 2
    wheel *= np.array([1.5, .55, 1.5]) / (high-low)
    radii = np.linalg.norm(wheel[:, :, [0, 2]], axis=2)
    rim = radii.max(axis=1) < .335
    write('wheel_tire.STL', wheel[~rim])
    write('wheel_rim.STL', wheel[rim])
    manifest = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in ROOT.glob('*.STL')}
    (OUT / 'source_sha256.json').write_text(json.dumps(manifest, indent=2)+'\n', encoding='utf-8')


if __name__ == '__main__':
    main()
