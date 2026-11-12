"""Simple Mindlin plate FEM for deck/base plate thickness screening (not a certified FEA).

Q4 Mindlin plate, 2x2 bending / 1-point shear integration, optional Timoshenko
stiffener beams on mesh lines (non-composite, eccentricity ignored -> conservative).
Units: mm, N, MPa.
"""
import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.linalg import spsolve

E_STEEL, NU = 210000.0, 0.3


class Plate:
    def __init__(self, xs, ys, thick_fn, inside_fn=lambda x, y: True, E=E_STEEL, nu=NU):
        self.xs, self.ys = np.asarray(xs), np.asarray(ys)
        nx, ny = len(xs), len(ys)
        self.nid = -np.ones((nx, ny), int)
        self.elems = []
        for i in range(nx - 1):
            for j in range(ny - 1):
                cx, cy = (xs[i] + xs[i+1]) / 2, (ys[j] + ys[j+1]) / 2
                if inside_fn(cx, cy):
                    self.elems.append((i, j, thick_fn(cx, cy)))
        used = set()
        for i, j, _ in self.elems:
            used |= {(i, j), (i+1, j), (i+1, j+1), (i, j+1)}
        self.coords = []
        for k, (i, j) in enumerate(sorted(used)):
            self.nid[i, j] = k
            self.coords.append((xs[i], ys[j]))
        self.coords = np.array(self.coords)
        self.n = len(self.coords)
        self.E, self.nu = E, nu
        self.rows, self.cols, self.vals = [], [], []
        self.F = np.zeros(3 * self.n)
        self.fixed = set()
        self._assemble()

    def _assemble(self):
        E, nu = self.E, self.nu
        g = 1 / np.sqrt(3)
        for i, j, t in self.elems:
            a = (self.xs[i+1] - self.xs[i]) / 2
            b = (self.ys[j+1] - self.ys[j]) / 2
            nodes = [self.nid[i, j], self.nid[i+1, j], self.nid[i+1, j+1], self.nid[i, j+1]]
            Db = E * t**3 / (12 * (1 - nu**2)) * np.array([[1, nu, 0], [nu, 1, 0], [0, 0, (1 - nu) / 2]])
            Ds = 5 / 6 * E / (2 * (1 + nu)) * t * np.eye(2)
            K = np.zeros((12, 12))
            xi_n = np.array([-1, 1, 1, -1]); et_n = np.array([-1, -1, 1, 1])
            def shape(xi, et):
                N = (1 + xi * xi_n) * (1 + et * et_n) / 4
                dx = xi_n * (1 + et * et_n) / 4 / a
                dy = et_n * (1 + xi * xi_n) / 4 / b
                return N, dx, dy
            for xi in (-g, g):
                for et in (-g, g):
                    N, dx, dy = shape(xi, et)
                    B = np.zeros((3, 12))
                    for k in range(4):  # dof order w, bx, by
                        B[0, 3*k+1] = dx[k]; B[1, 3*k+2] = dy[k]
                        B[2, 3*k+1] = dy[k]; B[2, 3*k+2] = dx[k]
                    K += B.T @ Db @ B * a * b
            N, dx, dy = shape(0, 0)
            S = np.zeros((2, 12))
            for k in range(4):
                S[0, 3*k] = dx[k]; S[0, 3*k+1] = N[k]
                S[1, 3*k] = dy[k]; S[1, 3*k+2] = N[k]
            K += S.T @ Ds @ S * 4 * a * b
            dofs = np.array([[3*n, 3*n+1, 3*n+2] for n in nodes]).ravel()
            self._add(dofs, K)

    def _add(self, dofs, K):
        r, c = np.meshgrid(dofs, dofs, indexing='ij')
        self.rows.append(r.ravel()); self.cols.append(c.ravel()); self.vals.append(K.ravel())

    def near(self, x, y, r):
        d = np.hypot(self.coords[:, 0] - x, self.coords[:, 1] - y)
        return np.where(d <= r + 1e-9)[0]

    def box_nodes(self, x, y, hx, hy):
        c = self.coords
        return np.where((abs(c[:, 0] - x) <= hx + 1e-9) & (abs(c[:, 1] - y) <= hy + 1e-9))[0]

    def force(self, x, y, fz, r=8):
        ids = self.near(x, y, r)
        self.F[3 * ids] += fz / len(ids)

    def spring(self, ids, k_total):
        for n in ids:
            self._add(np.array([3*n]), np.array([[k_total / len(ids)]]))

    def fix_w(self, ids):
        self.fixed |= {3 * int(n) for n in ids}

    def beam(self, axis, at, lo, hi, EI, GA=None):
        """Stiffener along mesh line. axis='x': beam along x at y=at."""
        E_al = 70000.0
        GA = GA or 0.4 * 26000 * 150
        if axis == 'x':
            j = int(np.argmin(abs(self.ys - at)))
            line = [(i, j) for i in range(len(self.xs)) if lo - 1e-9 <= self.xs[i] <= hi + 1e-9]
            rot = 1
        else:
            i = int(np.argmin(abs(self.xs - at)))
            line = [(i, j) for j in range(len(self.ys)) if lo - 1e-9 <= self.ys[j] <= hi + 1e-9]
            rot = 2
        for (p, q) in zip(line[:-1], line[1:]):
            n1, n2 = self.nid[p], self.nid[q]
            if n1 < 0 or n2 < 0:
                continue
            L = np.hypot(*(self.coords[n2] - self.coords[n1]))
            Kb = EI / L * np.array([[0, 0, 0, 0], [0, 1, 0, -1], [0, 0, 0, 0], [0, -1, 0, 1]])
            s = np.array([-1 / L, 0.5, 1 / L, 0.5])
            Ks = GA * L * np.outer(s, s)
            dofs = np.array([3*n1, 3*n1+rot, 3*n2, 3*n2+rot])
            self._add(dofs, Kb + Ks)

    def solve(self):
        nd = 3 * self.n
        K = coo_matrix((np.concatenate(self.vals), (np.concatenate(self.rows), np.concatenate(self.cols))),
                       shape=(nd, nd)).tocsr()
        free = np.array(sorted(set(range(nd)) - self.fixed))
        u = np.zeros(nd)
        u[free] = spsolve(K[free][:, free].tocsc(), self.F[free])
        self.u = u
        return u

    def stresses(self, t_share_fn=None):
        """Max von Mises surface stress per element (element centre) of the plate itself."""
        E, nu = self.E, self.nu
        out = []
        for i, j, t in self.elems:
            a = (self.xs[i+1] - self.xs[i]) / 2; b = (self.ys[j+1] - self.ys[j]) / 2
            nodes = [self.nid[i, j], self.nid[i+1, j], self.nid[i+1, j+1], self.nid[i, j+1]]
            xi_n = np.array([-1, 1, 1, -1]); et_n = np.array([-1, -1, 1, 1])
            dx = xi_n / 4 / a; dy = et_n / 4 / b
            bx = self.u[[3*n+1 for n in nodes]]; by = self.u[[3*n+2 for n in nodes]]
            kx, ky, kxy = dx @ bx, dy @ by, dy @ bx + dx @ by
            cx, cy = (self.xs[i] + self.xs[i+1]) / 2, (self.ys[j] + self.ys[j+1]) / 2
            ts = t_share_fn(cx, cy) if t_share_fn else t  # skin thickness for stress
            z = ts / 2
            c = E / (1 - nu**2)
            sx = c * z * (kx + nu * ky); sy = c * z * (ky + nu * kx); txy = E / (2 * (1 + nu)) * z * kxy
            out.append((np.sqrt(sx**2 - sx*sy + sy**2 + 3*txy**2), cx, cy))
        return out
