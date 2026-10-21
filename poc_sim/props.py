"""Procedural visual mesh for the pallet.

The tote and the gripper are now built from measured dimensions as plain boxes in
`tote.py` and `gripper.py`, where the same boxes serve as both collision and what
the camera sees. Only the pallet still wants a mesh, because its lattice is scenery
rather than anything the robot interacts with.
"""

import math
import xml.etree.ElementTree as E
from pathlib import Path

import numpy as np

from .layout import PALLET_HALF_X, PALLET_HALF_Y, PALLET_TOP

PROP_DIR = Path(__file__).resolve().parents[1] / "assets" / "props"



class Mesh:
    def __init__(self):
        self.vertices = []
        self.faces = []

    def quad(self, a, b, c, d):
        self.faces.extend([(a, b, c), (a, c, d)])

    def box(self, center, half):
        start = len(self.vertices)
        c, h = np.array(center), np.array(half)
        self.vertices.extend(
            (c + h * np.array(v)).tolist()
            for v in [
                (-1, -1, -1),
                (1, -1, -1),
                (1, 1, -1),
                (-1, 1, -1),
                (-1, -1, 1),
                (1, -1, 1),
                (1, 1, 1),
                (-1, 1, 1),
            ]
        )
        for f in [
            (0, 3, 2, 1),
            (4, 5, 6, 7),
            (0, 1, 5, 4),
            (1, 2, 6, 5),
            (2, 3, 7, 6),
            (3, 0, 4, 7),
        ]:
            self.quad(*(start + i for i in f))

    def slanted_box(self, center, half, axis, slope):
        start = len(self.vertices)
        self.box(center, half)
        for v in self.vertices[start:]:
            v[axis] += slope * (v[2] - center[2])

    def loft(self, axis, sections):
        """Rectangular cross-sections swept along one axis, then skinned."""
        u, v = [k for k in range(3) if k != axis]
        start = len(self.vertices)
        for t, cu, cv, hu, hv in sections:
            for su, sv in [(-1, -1), (1, -1), (1, 1), (-1, 1)]:
                p = [0.0, 0.0, 0.0]
                p[axis], p[u], p[v] = t, cu + su * hu, cv + sv * hv
                self.vertices.append(p)
        for k in range(len(sections) - 1):
            for j in range(4):
                a, b = start + k * 4 + j, start + k * 4 + (j + 1) % 4
                self.quad(a, b, b + 4, a + 4)
        last = start + (len(sections) - 1) * 4
        self.quad(start + 3, start + 2, start + 1, start)
        self.quad(last, last + 1, last + 2, last + 3)

    def ring(self, hx, hy, radius, z, inner_x, inner_y, inner_radius, height):
        loops = [
            rounded_rectangle(hx, hy, radius, z),
            rounded_rectangle(hx, hy, radius, z + height),
            rounded_rectangle(inner_x, inner_y, inner_radius, z + height),
            rounded_rectangle(inner_x, inner_y, inner_radius, z),
        ]
        start = len(self.vertices)
        n = len(loops[0])
        for loop in loops:
            self.vertices.extend(loop)
        for k in range(4):
            for j in range(n):
                a = start + k * n + j
                b = start + k * n + (j + 1) % n
                c = start + ((k + 1) % 4) * n + (j + 1) % n
                d = start + ((k + 1) % 4) * n + j
                self.quad(a, b, c, d)

    def rounded_slab(self, hx, hy, radius, z0, z1):
        loops = [
            rounded_rectangle(hx, hy, radius, z0),
            rounded_rectangle(hx, hy, radius, z1),
        ]
        n = len(loops[0])
        offset = len(self.vertices)
        for loop in loops:
            self.vertices.extend(loop)
        low = len(self.vertices)
        self.vertices.extend([(0, 0, z0), (0, 0, z1)])
        for j in range(n):
            k = (j + 1) % n
            self.quad(offset + j, offset + k, offset + n + k, offset + n + j)
            self.faces.append((low, offset + k, offset + j))
            self.faces.append((low + 1, offset + n + j, offset + n + k))

    def write(self, name):
        path = PROP_DIR / f"{name}.obj"
        text = "# Procedural visual mesh, metres; not manufacturer CAD.\n"
        text += "".join(
            "v " + " ".join(f"{x:.7f}" for x in v) + "\n" for v in self.vertices
        )
        text += "".join(
            "f " + " ".join(str(i + 1) for i in f) + "\n" for f in self.faces
        )
        if not path.exists() or path.read_text() != text:
            path.write_text(text)
        return path


def rounded_rectangle(hx, hy, radius, z):
    """CCW contour with rounded corners and subdivided flats (matching topology)."""
    points = []
    centers = [
        (hx - radius, hy - radius),
        (-hx + radius, hy - radius),
        (-hx + radius, -hy + radius),
        (hx - radius, -hy + radius),
    ]
    for k, (cx, cy) in enumerate(centers):
        for a in np.linspace(k * math.pi / 2, (k + 1) * math.pi / 2, 7):
            points.append((cx + radius * math.cos(a), cy + radius * math.sin(a), z))
        end = np.array(points[-1])
        nx, ny = centers[(k + 1) % 4]
        a = (k + 1) * math.pi / 2
        target = np.array([nx + radius * math.cos(a), ny + radius * math.sin(a), z])
        for f in np.linspace(0, 1, 13)[1:-1]:
            points.append(tuple(end + (target - end) * f))
    return points


def make_meshes():
    PROP_DIR.mkdir(parents=True, exist_ok=True)
    pallet = Mesh()
    # 1000 x 1200 x 123 mm; nine feet, three runners, open fork pockets.
    deck = PALLET_TOP - 0.023
    for x in [-0.43, 0, 0.43]:
        pallet.box([x, 0, 0.009], [0.07, PALLET_HALF_Y, 0.009])
        for y in [-0.52, 0, 0.52]:
            pallet.box([x, y, (0.018 + deck) / 2], [0.07, 0.08, (deck - 0.018) / 2])
    for y in [-0.565, 0.565]:
        pallet.box([0, y, 0.009], [PALLET_HALF_X, 0.035, 0.009])
    pallet.ring(PALLET_HALF_X, PALLET_HALF_Y, 0.018, deck, 0.472, 0.572, 0.006, 0.023)
    for x in [-0.3, -0.1, 0.1, 0.3]:
        pallet.box([x, 0, PALLET_TOP - 0.014], [0.012, 0.572, 0.014])
    for y in [-0.4, -0.2, 0, 0.2, 0.4]:
        pallet.box([0, y, PALLET_TOP - 0.014], [0.472, 0.012, 0.014])
    for x in np.linspace(-0.46, 0.46, 21):
        pallet.box([x, 0, PALLET_TOP - 0.007], [0.004, 0.572, 0.007])
    for y in np.linspace(-0.56, 0.56, 25):
        pallet.box([0, y, PALLET_TOP - 0.007], [0.472, 0.004, 0.007])
    return {"pallet_lattice": pallet.write("pallet_lattice")}


def add_assets(asset):
    for name, path in make_meshes().items():
        E.SubElement(asset, "mesh", name=name, file=str(path))
    for name, rgba in [("pallet_plastic", ".035 .26 .13 1")]:
        E.SubElement(
            asset,
            "material",
            name=name,
            rgba=rgba,
            specular=".32",
            shininess=".25",
            reflectance=".02",
        )


def visual_geom(parent, **kwargs):
    return E.SubElement(
        parent, "geom", contype="0", conaffinity="0", mass="0", group="1", **kwargs
    )


def add_pallet_visual(world, pos=(1.15, 0, 0)):
    visual_geom(
        world,
        name="pallet_visual",
        type="mesh",
        mesh="pallet_lattice",
        pos=" ".join(map(str, pos)),
        material="pallet_plastic",
    )
