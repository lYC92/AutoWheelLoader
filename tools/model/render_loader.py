"""Render expanded URDF visuals using Blender's CPU renderer (no Gazebo needed).

blender --background --threads 2 --python tools/model/render_loader.py -- 
  --urdf results/model/loader_review.urdf --output results/model/loader.png
Mesh filenames are resolved against this repository's loader_description package.
"""
import argparse
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

import bpy
import numpy as np
from mathutils import Euler, Matrix, Vector


def origin(element):
    if element is None:
        return Matrix.Identity(4)
    xyz = [float(v) for v in element.get('xyz', '0 0 0').split()]
    rpy = [float(v) for v in element.get('rpy', '0 0 0').split()]
    return Matrix.Translation(xyz) @ Euler(rpy, 'XYZ').to_matrix().to_4x4()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--urdf', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--lift', type=float, default=0)
    parser.add_argument('--tilt', type=float, default=0)
    parser.add_argument('--steer', type=float, default=0)
    parser.add_argument('--side', action='store_true')
    args = parser.parse_args(sys.argv[sys.argv.index('--') + 1:])
    root = ET.parse(args.urdf).getroot()
    package = Path(__file__).resolve().parents[2] / 'ros_ws/src/loader_description'
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)
    materials = {}
    for element in root.findall('material'):
        mat = bpy.data.materials.new(element.get('name'))
        mat.diffuse_color = tuple(float(v) for v in element.find('color').get('rgba').split())
        mat.use_nodes = True
        bsdf = mat.node_tree.nodes.get('Principled BSDF')
        bsdf.inputs['Base Color'].default_value = mat.diffuse_color
        bsdf.inputs['Roughness'].default_value = 0.48
        materials[element.get('name')] = mat
    poses = {'base_link': Matrix.Translation((0, 0, 0.15))}
    angles = {'lift_joint': args.lift, 'bucket_tilt_joint': args.tilt, 'articulation_joint': args.steer}
    pending = list(root.findall('joint'))
    while pending:
        ready = [j for j in pending if j.find('parent').get('link') in poses]
        if not ready:
            raise ValueError('URDF joint graph cannot be resolved')
        for joint in ready:
            transform = origin(joint.find('origin'))
            angle = angles.get(joint.get('name'), 0)
            if angle:
                axis = Vector(tuple(float(v) for v in joint.find('axis').get('xyz').split()))
                transform @= Matrix.Rotation(angle, 4, axis)
            poses[joint.find('child').get('link')] = poses[joint.find('parent').get('link')] @ transform
            pending.remove(joint)
    stl_type = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attr', '<u2')])
    for link in root.findall('link'):
        for visual in link.findall('visual'):
            geometry = visual.find('geometry')
            mesh_element = geometry.find('mesh')
            if mesh_element is not None:
                path = package / 'meshes' / mesh_element.get('filename').split('/meshes/')[1]
                triangles = np.fromfile(path, dtype=stl_type, offset=84)['vertices']
                mesh = bpy.data.meshes.new(link.get('name'))
                mesh.from_pydata(triangles.reshape(-1, 3).tolist(), [], np.arange(triangles.size // 3).reshape(-1, 3).tolist())
                mesh.update()
                obj = bpy.data.objects.new(link.get('name'), mesh)
                bpy.context.collection.objects.link(obj)
                scale = tuple(float(v) for v in mesh_element.get('scale', '1 1 1').split())
            elif geometry.find('box') is not None:
                bpy.ops.mesh.primitive_cube_add(size=1)
                obj = bpy.context.object
                scale = tuple(float(v) for v in geometry.find('box').get('size').split())
            elif geometry.find('cylinder') is not None:
                cylinder = geometry.find('cylinder')
                bpy.ops.mesh.primitive_cylinder_add(vertices=48, radius=float(cylinder.get('radius')), depth=float(cylinder.get('length')))
                obj, scale = bpy.context.object, (1, 1, 1)
            else:
                continue
            obj.name = link.get('name')
            obj.matrix_world = poses[link.get('name')] @ origin(visual.find('origin')) @ Matrix.Diagonal((*scale, 1))
            material = visual.find('material')
            if material is not None and material.get('name') in materials:
                obj.data.materials.append(materials[material.get('name')])
    bpy.ops.mesh.primitive_plane_add(size=200)
    ground = bpy.context.object
    groundmat = bpy.data.materials.new('ground')
    groundmat.diffuse_color = (.22, .26, .31, 1)
    ground.data.materials.append(groundmat)
    scene = bpy.context.scene
    scene.world.color = (.3, .3, .3)
    for pos, energy, size in [((2, -5, 10), 2100, 7), ((-4, 3, 7), 1700, 6)]:
        bpy.ops.object.light_add(type='AREA', location=pos)
        light = bpy.context.object
        light.data.energy, light.data.shape, light.data.size = energy, 'DISK', size
        light.rotation_euler = (Vector((0, 0, 1)) - light.location).to_track_quat('-Z', 'Y').to_euler()
    bpy.ops.object.camera_add(location=(.8, -15, 4.0) if args.side else (11, -13, 8))
    camera = bpy.context.object
    camera.rotation_euler = (Vector((.7, 0, 1.6)) - camera.location).to_track_quat('-Z', 'Y').to_euler()
    camera.data.type, camera.data.ortho_scale = 'ORTHO', 11.6
    scene.camera = camera
    scene.render.engine = 'CYCLES'
    scene.cycles.device = 'CPU'
    scene.cycles.samples = 16
    scene.render.threads_mode, scene.render.threads = 'FIXED', 2
    scene.render.resolution_x, scene.render.resolution_y = 1200, 800
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = 'PNG'
    output = Path(args.output).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    scene.render.filepath = str(output)
    bpy.ops.render.render(write_still=True)


if __name__ == '__main__':
    main()
