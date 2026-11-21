#!/usr/bin/env python3
"""OFFLINE tool (not part of the runtime): bake the MJCF into a MuJoCo-independent model file.

    python3 tools/export_robot_model.py --mjcf mjcf/scene.xml --out config/robot_model.txt

The C++ core (arm_core) only reads robot_model.txt, so the same kinematics / collision code
serves the simulation and the real robot. The file holds
  * a body tree: world + one body per joint (fixed bodies such as base_link, esp32_cam, hcsr04
    are merged into the body they are rigidly attached to),
  * a convex hull per collision geom (vertices, face normals, edge directions),
  * named frames (tool = grasp_center, hcsr04 sensor, camera) expressed in a body.
Everything is read from the MJCF: no copied numbers. Re-run after any change to arm.xml.
Needs numpy + scipy.
"""
import argparse, os, struct, sys
import numpy as np
import xml.etree.ElementTree as ET
from scipy.spatial import ConvexHull


def quat_to_mat(q):
    w, x, y, z = np.array(q, float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def T_of(pos, quat):
    M = np.eye(4); M[:3, :3] = quat_to_mat(quat); M[:3, 3] = pos; return M


def fl(e, k, d):
    return [float(x) for x in e.get(k, d).split()]


def read_stl(path):
    b = open(path, 'rb').read()
    n = struct.unpack('<I', b[80:84])[0]
    if 84 + n * 50 == len(b):
        dt = np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')])
        v = np.frombuffer(b[84:], dtype=dt, count=n)['v'].reshape(-1, 3).astype(float)
    else:
        v = np.array([[float(x) for x in l.split()[1:4]] for l in b.decode().splitlines()
                      if l.strip().startswith('vertex')])
    return np.unique(v.round(8), axis=0)


def box_pts(s):
    return np.array([[a * s[0], b * s[1], c * s[2]] for a in (-1, 1) for b in (-1, 1) for c in (-1, 1)])


def cyl_pts(r, hh, n=24):
    t = np.linspace(0, 2 * np.pi, n, endpoint=False)
    return np.array([[r * np.cos(a), r * np.sin(a), z] for z in (-hh, hh) for a in t])


def hull_data(pts):
    h = ConvexHull(pts)
    P = pts[h.vertices]
    h = ConvexHull(P)
    P = P[h.vertices]
    h = ConvexHull(P)
    normals = np.unique(h.equations[:, :3].round(6), axis=0)
    edges = set()
    for s in h.simplices:
        for a, b in ((0, 1), (1, 2), (0, 2)):
            edges.add((min(s[a], s[b]), max(s[a], s[b])))
    E = np.array([P[b] - P[a] for a, b in edges])
    E /= np.linalg.norm(E, axis=1, keepdims=True)
    for i in range(len(E)):                       # canonical sign so +e and -e are one direction
        k = np.argmax(np.abs(E[i]) > 1e-9)
        if E[i][k] < 0: E[i] = -E[i]
    E = np.unique(E.round(5), axis=0)
    return P, normals, E


class Exporter:
    def __init__(self, mjcf_scene):
        self.dir = os.path.dirname(os.path.abspath(mjcf_scene))
        self.meshdir = os.path.join(self.dir, '..', 'meshes')
        self.scene = ET.parse(mjcf_scene).getroot()
        inc = self.scene.find('include')
        self.arm = ET.parse(os.path.join(self.dir, inc.get('file'))).getroot() if inc is not None else self.scene
        self.mesh_file = {m.get('name'): m.get('file') for r in (self.arm, self.scene) for m in r.iter('mesh')}
        # actuated joints -> dof index (actuator order), coupled joints from <equality>
        self.dof = {}
        for a in self.arm.find('actuator'):
            self.dof.setdefault(a.get('joint'), len(self.dof))
        self.coef = {j: (self.dof[j], 1.0) for j in self.dof}
        eq = self.arm.find('equality')
        if eq is not None:
            for e in eq.findall('joint'):
                pc = fl(e, 'polycoef', '0 1 0 0 0')
                assert abs(pc[0]) < 1e-12 and all(abs(c) < 1e-12 for c in pc[2:]), 'only linear joint coupling supported'
                self.coef[e.get('joint1')] = (self.dof[e.get('joint2')], pc[1])
        self.bodies = [dict(name='world', parent=-1, fixed=np.eye(4), dof=-1, coef=0.0, axis=[0, 0, 1], lo=0, hi=0)]
        self.hulls, self.frames = [], []
        self.arm_names = []
        self._walk(self.arm.find('worldbody').find('body'), 0, np.eye(4))
        # static scene geoms attached to the world body (mount plate)
        for g in self.scene.find('worldbody').findall('geom'):
            if g.get('name') == 'mount_plate':
                self._add_hull('mount_plate', 0, box_pts(fl(g, 'size', '0 0 0')), T_of(fl(g, 'pos', '0 0 0'), [1, 0, 0, 0]))

    def _add_hull(self, name, body, pts, Tg):
        V = pts @ Tg[:3, :3].T + Tg[:3, 3]
        P, N, E = hull_data(V)
        self.hulls.append(dict(name=name, body=body, P=P, N=N, E=E))

    def _walk(self, b, parent_mb, T_parent_rel):
        j = b.find('joint')
        T_local = T_of(fl(b, 'pos', '0 0 0'), fl(b, 'quat', '1 0 0 0'))
        if j is None:
            mb, T_rel = parent_mb, T_parent_rel @ T_local
        else:
            jn = j.get('name'); d, c = self.coef[jn]
            ax = np.array(fl(j, 'axis', '0 0 1')); ax /= np.linalg.norm(ax)
            lim = fl(j, 'range', '0 0')
            self.bodies.append(dict(name=b.get('name'), parent=parent_mb, fixed=T_parent_rel @ T_local,
                                    dof=d, coef=c, axis=list(ax), lo=lim[0], hi=lim[1], joint=jn))
            mb, T_rel = len(self.bodies) - 1, np.eye(4)
        for k, g in enumerate(b.findall('geom')):
            if g.get('class') == 'vis':
                continue
            if g.get('mesh'):
                pts = read_stl(os.path.join(self.meshdir, self.mesh_file[g.get('mesh')])); name = g.get('mesh')
            elif g.get('type') == 'box':
                pts = box_pts(fl(g, 'size', '0 0 0')); name = '%s_g%d' % (b.get('name'), k)
            elif g.get('type') == 'cylinder':
                s = fl(g, 'size', '0 0'); pts = cyl_pts(s[0], s[1]); name = '%s_g%d' % (b.get('name'), k)
            else:
                continue
            self._add_hull(name, mb, pts, T_rel @ T_of(fl(g, 'pos', '0 0 0'), fl(g, 'quat', '1 0 0 0')))
        for s in b.findall('site'):
            self.frames.append((s.get('name'), mb, T_rel @ T_of(fl(s, 'pos', '0 0 0'), fl(s, 'quat', '1 0 0 0'))))
        for c in b.findall('camera'):
            self.frames.append(('cam_' + c.get('name'), mb, T_rel @ T_of(fl(c, 'pos', '0 0 0'), fl(c, 'quat', '1 0 0 0'))))
        for c in b.findall('body'):
            self._walk(c, mb, T_rel)

    def write(self, path, model_name):
        # tool frame: grasp_center position, orientation = revolute_4 frame re-labelled so that the identity
        # pose is "gripper horizontal along +x, y left, z up" (ROS convention): x_tool = x4, y_tool = -z4, z_tool = y4.
        R_tool = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]], float)
        out = []
        out.append('# robot model baked from the MJCF by tools/export_robot_model.py - DO NOT EDIT BY HAND')
        out.append('# units: metres / radians. Rotations are row-major 3x3; quaternions are (w x y z).')
        out.append('model %s' % model_name)
        out.append('ndof %d' % (max(self.dof.values()) + 1))
        for b in self.bodies:
            Tf = b['fixed']
            R = Tf[:3, :3].reshape(-1)
            out.append('body %s parent %d pos %s rot %s dof %d coef %.9g axis %s limits %.9g %.9g' % (
                b['name'], b['parent'], ' '.join('%.9g' % x for x in Tf[:3, 3]), ' '.join('%.9g' % x for x in R),
                b['dof'], b['coef'], ' '.join('%.9g' % x for x in b['axis']), b['lo'], b['hi']))
        frames = list(self.frames)
        site = {n: (bd, T) for n, bd, T in frames}
        bd, Tg = site['grasp_center']
        Tt = Tg.copy(); Tt[:3, :3] = R_tool
        frames.insert(0, ('tool', bd, Tt))
        for n, bd, T in frames:
            out.append('frame %s body %d pos %s rot %s' % (n, bd, ' '.join('%.9g' % x for x in T[:3, 3]),
                                                           ' '.join('%.9g' % x for x in T[:3, :3].reshape(-1))))
        for h in self.hulls:
            out.append('hull %s body %d nv %d nn %d ne %d' % (h['name'], h['body'], len(h['P']), len(h['N']), len(h['E'])))
            for arr in (h['P'], h['N'], h['E']):
                for r in arr:
                    out.append('%.9g %.9g %.9g' % tuple(r))
        open(path, 'w').write('\n'.join(out) + '\n')
        return len(self.bodies), len(self.hulls), len(frames)


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--mjcf', required=True)
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    ex = Exporter(a.mjcf)
    nb, nh, nf = ex.write(a.out, ex.arm.get('model', 'robot'))
    print('wrote %s: %d bodies, %d hulls, %d frames' % (a.out, nb, nh, nf))
