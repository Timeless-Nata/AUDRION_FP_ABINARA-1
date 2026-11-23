#!/usr/bin/env python3
"""DEV tool: independent numpy reference for the C++ unit tests.

Reads the MJCF directly (NOT robot_model.txt) and computes, for random joint vectors,
  * tool pose by forward kinematics,
  * the set of overlapping hull pairs + penetration depths (numpy SAT, MuJoCo contact-filter rules),
and writes test/reference_vectors.txt. The C++ core must reproduce these numbers from robot_model.txt,
which therefore validates the exporter, the loader, FK and the collision checker together.
    python3 tools/make_reference_vectors.py --mjcf mjcf/scene.xml --out test/reference_vectors.txt
"""
import argparse, itertools, os, sys
import numpy as np
import xml.etree.ElementTree as ET
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from export_robot_model import quat_to_mat, T_of, fl, read_stl, box_pts, cyl_pts, hull_data
from scipy.spatial import ConvexHull


def Rot(axis, ang):
    a = np.array(axis, float); a /= np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    M = np.eye(4); M[:3, :3] = np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * K @ K; return M


class Ref:
    def __init__(self, scene_path):
        d = os.path.dirname(os.path.abspath(scene_path)); self.meshdir = os.path.join(d, '..', 'meshes')
        sc = ET.parse(scene_path).getroot(); self.arm = ET.parse(os.path.join(d, sc.find('include').get('file'))).getroot()
        self.mesh_file = {m.get('name'): m.get('file') for r in (self.arm, sc) for m in r.iter('mesh')}
        self.bodies = {'world': dict(parent=None, T=np.eye(4), joint=None, geoms=[])}
        self._walk(self.arm.find('worldbody').find('body'), 'world')
        for g in sc.find('worldbody').findall('geom'):
            if g.get('name') == 'mount_plate':
                self.bodies['world']['geoms'].append(('mount_plate', box_pts(fl(g, 'size', '0 0 0')), T_of(fl(g, 'pos', '0 0 0'), [1, 0, 0, 0])))

    def _walk(self, b, par):
        n = b.get('name'); j = b.find('joint')
        self.bodies[n] = dict(parent=par, T=T_of(fl(b, 'pos', '0 0 0'), fl(b, 'quat', '1 0 0 0')),
                              joint=None if j is None else (j.get('name'), fl(j, 'axis', '0 0 1')), geoms=[])
        for k, g in enumerate(b.findall('geom')):
            if g.get('class') == 'vis': continue
            if g.get('mesh'): pts = read_stl(os.path.join(self.meshdir, self.mesh_file[g.get('mesh')])); gn = g.get('mesh')
            elif g.get('type') == 'box': pts = box_pts(fl(g, 'size', '0 0 0')); gn = '%s_g%d' % (n, k)
            elif g.get('type') == 'cylinder': s = fl(g, 'size', '0 0'); pts = cyl_pts(s[0], s[1]); gn = '%s_g%d' % (n, k)
            else: continue
            self.bodies[n]['geoms'].append((gn, pts, T_of(fl(g, 'pos', '0 0 0'), fl(g, 'quat', '1 0 0 0'))))
        for c in b.findall('body'): self._walk(c, n)

    def fk_body(self, n, q):
        chain = []
        while n != 'world': chain.append(n); n = self.bodies[n]['parent']
        M = np.eye(4)
        for b in reversed(chain):
            M = M @ self.bodies[b]['T']; j = self.bodies[b]['joint']
            if j and j[0] in q: M = M @ Rot(j[1], q[j[0]])
        return M

    def tool(self, q):
        M = self.fk_body('revolute_4', q)
        site = [s for s in self.bodies and self.arm.iter('site') if s.get('name') == 'grasp_center'][0]
        p = M @ np.append(fl(site, 'pos', '0 0 0'), 1)
        Rt = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]], float)     # tool axes: x=x4, y=-z4, z=y4
        return p[:3], M[:3, :3] @ Rt

    def weld(self, n):
        d = self.bodies[n]
        if n == 'world': return 'world'
        if d['joint'] is None: return self.weld(d['parent'])
        return n

    def filtered(self, a, b):
        wa, wb = self.weld(a), self.weld(b)
        if wa == wb: return True
        pa = 'world' if wa == 'world' else self.weld(self.bodies[wa]['parent'])
        pb = 'world' if wb == 'world' else self.weld(self.bodies[wb]['parent'])
        return wa != 'world' and wb != 'world' and (pa == wb or pb == wa)

    @staticmethod
    def sat(A, B):
        (PA, nA, eA), (PB, nB, eB) = A, B
        cr = np.cross(eA[:, None, :], eB[None, :, :]).reshape(-1, 3); L = np.linalg.norm(cr, axis=1)
        axes = np.vstack([nA, nB, cr[L > 1e-6] / L[L > 1e-6, None]]); pa = PA @ axes.T; pb = PB @ axes.T
        return np.minimum(pa.max(0) - pb.min(0), pb.max(0) - pa.min(0)).min()

    def collisions(self, q):
        H = []
        for n, d in self.bodies.items():
            M = self.fk_body(n, q)
            for gn, pts, Tg in d['geoms']:
                G = M @ Tg; V = pts @ G[:3, :3].T + G[:3, 3]; H.append((n, gn, V))
        res = {}
        for n, gn, V in H:
            if self.weld(n) != 'world' and V[:, 2].min() < 0: res[('FLOOR', gn)] = -V[:, 2].min()
        prepped = {}
        for (n1, g1, V1), (n2, g2, V2) in itertools.combinations(H, 2):
            if self.filtered(n1, n2): continue
            if (V1.min(0) > V2.max(0)).any() or (V2.min(0) > V1.max(0)).any(): continue
            for g, V in ((g1, V1), (g2, V2)):
                if g not in prepped:
                    h = ConvexHull(V); P = V[h.vertices]; P2, nrm, ed = hull_data(P); prepped[g] = (P2, nrm, ed)
            d = self.sat(prepped[g1], prepped[g2])
            if d > 0: res[tuple(sorted((g1, g2)))] = d
        return res


if __name__ == '__main__':
    ap = argparse.ArgumentParser(); ap.add_argument('--mjcf', required=True); ap.add_argument('--out', required=True)
    ap.add_argument('--n-fk', type=int, default=30); ap.add_argument('--n-col', type=int, default=60)
    a = ap.parse_args()
    R = Ref(a.mjcf); rng = np.random.default_rng(12345)
    lim = np.array([[-1.571, 1.571]] * 4)
    out = ['# generated by tools/make_reference_vectors.py from the MJCF (independent numpy implementation)',
           '# fk  q1 q2 q3 q4  px py pz  R(row-major 9)',
           '# col q1 q2 q3 q4 finger  npairs  (pairs "a|b=depth" ...)']
    names = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
    for _ in range(a.n_fk):
        q = rng.uniform(lim[:, 0], lim[:, 1]); p, Rm = R.tool(dict(zip(names, q)))
        out.append('fk ' + ' '.join('%.9f' % x for x in q) + ' ' + ' '.join('%.9f' % x for x in p) + ' ' + ' '.join('%.9f' % x for x in Rm.reshape(-1)))
    got = 0; ncol = 0
    while got < a.n_col:
        q = rng.uniform(lim[:, 0], lim[:, 1]); fa = rng.uniform(-0.873, 0.227)
        # bias half of the samples to the neighbourhood of the home pose, where collisions are borderline
        if got % 2 == 0: q = np.array([0, 0.8901, 0.4538, -0.4363]) + rng.normal(0, 0.4, 4); q = np.clip(q, lim[:, 0], lim[:, 1])
        qd = dict(zip(names, q)); qd['finger_active'] = fa; qd['finger_passive'] = -fa
        r = R.collisions(qd)
        if any(abs(d) < 1e-4 for d in r.values()):   # grazing contact: numerically ambiguous, skip sample
            continue
        ncol += bool(r)
        out.append('col ' + ' '.join('%.9f' % x for x in q) + ' %.9f %d ' % (fa, len(r)) + ' '.join('%s|%s=%.9f' % (k[0], k[1], v) for k, v in sorted(r.items())))
        got += 1
    open(a.out, 'w').write('\n'.join(out) + '\n')
    print('wrote %s: %d fk, %d collision samples (%d with overlaps)' % (a.out, a.n_fk, a.n_col, ncol))
