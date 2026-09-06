"""Point-cloud geometry without ROS or vehicle ground truth."""
from pathlib import Path
from urllib.parse import unquote, urlparse
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation


def range_mask(xyz, minimum, maximum):
    radius = np.linalg.norm(xyz, axis=1)
    return np.isfinite(xyz).all(axis=1) & (radius > minimum) & (radius < maximum)


def ground_plane(xyz, threshold=.18, max_tilt_deg=30., seed=7):
    """Fit broad ground below the sensor; reject walls/lines without world pose."""
    candidate = xyz[np.isfinite(xyz).all(axis=1) & (xyz[:, 2] < -.8)]
    if len(candidate) < 60:
        return None
    rng = np.random.default_rng(seed)
    sample = candidate[rng.choice(len(candidate), min(768, len(candidate)), replace=False)].astype(float)
    best, count = None, 0
    cosine = np.cos(np.radians(max_tilt_deg))
    for _ in range(64):
        a, b, c = sample[rng.choice(len(sample), 3, replace=False)]
        normal = np.cross(b-a, c-a)
        length = np.linalg.norm(normal)
        if length < 1e-8:
            continue
        normal /= length
        if normal[2] < 0:
            normal *= -1
        offset = -normal @ a
        if normal[2] < cosine or not .8 < offset / normal[2] < 6.:
            continue
        inliers = abs(sample @ normal + offset) < threshold
        if inliers.sum() > count:
            best, count = inliers, int(inliers.sum())
    if best is None or count < max(40, .2 * len(sample)):
        return None
    points = sample[best]
    center = points.mean(axis=0)
    eigenvalues, eigenvectors = np.linalg.eigh(np.cov(points-center, rowvar=False))
    if eigenvalues[1] < 1.:
        return None
    normal = eigenvectors[:, 0]
    if normal[2] < 0:
        normal *= -1
    offset = -normal @ center
    if normal[2] < cosine or not .8 < offset / normal[2] < 6.:
        return None
    return normal, offset


def outside_body(xyz, bounds, transforms, margin=.10):
    """Keep returns outside visual boxes, using lidar_from_link transforms."""
    keep = np.ones(len(xyz), dtype=bool)
    for link, origin, low, high in bounds:
        transform = transforms[link] @ origin
        local = (xyz-transform[:3, 3]) @ transform[:3, :3]
        keep &= ~((local >= low-margin) & (local <= high+margin)).all(axis=1)
    return keep


def visual_bounds(urdf):
    """Read rendered extents: collision boxes omit cab and bucket lips."""
    bounds = []
    root = ET.parse(urdf).getroot()
    dtype = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attr', '<u2')])
    for link in root.findall('link'):
        for visual in link.findall('visual'):
            geom = visual.find('geometry')
            origin = np.eye(4)
            element = visual.find('origin')
            if element is not None:
                origin[:3, 3] = np.fromstring(element.get('xyz', '0 0 0'), sep=' ')
                origin[:3, :3] = Rotation.from_euler('xyz', np.fromstring(element.get('rpy', '0 0 0'), sep=' ')).as_matrix()
            if geom.find('mesh') is not None:
                mesh = geom.find('mesh')
                uri = urlparse(mesh.get('filename'))
                if uri.scheme != 'file':
                    raise ValueError(f'expected expanded file mesh URI, got {uri.scheme}')
                path = Path(unquote(uri.path))
                data = path.read_bytes()
                count = int.from_bytes(data[80:84], 'little')
                if len(data) != 84 + 50*count or count == 0:
                    raise ValueError(f'invalid binary STL: {path}')
                vertices = np.frombuffer(data, dtype=dtype, offset=84)['vertices'].reshape(-1, 3)
                vertices = vertices * np.fromstring(mesh.get('scale', '1 1 1'), sep=' ')
                low, high = vertices.min(axis=0), vertices.max(axis=0)
            elif geom.find('box') is not None:
                high = np.fromstring(geom.find('box').get('size'), sep=' ') / 2
                low = -high
            elif geom.find('cylinder') is not None:
                cylinder = geom.find('cylinder')
                radius = float(cylinder.get('radius'))
                high = np.array([radius, radius, float(cylinder.get('length'))/2])
                low = -high
            else:
                raise ValueError('unsupported visual geometry in self mask')
            if not np.isfinite([low, high, origin[:3, 3]]).all():
                raise ValueError('non-finite model geometry')
            bounds.append((link.get('name'), origin, low, high))
    return bounds
