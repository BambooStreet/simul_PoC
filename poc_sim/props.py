"""Procedural visual meshes for a photo-referenced industrial nesting tote/pallet.

Dimensions are supplied by the user; details are approximations, not vendor CAD.
All visual geoms are massless and noncolliding. The scene owns contact proxies.
"""

import math
import xml.etree.ElementTree as E
from pathlib import Path

import numpy as np

from .layout import (
    HANDLE_BOTTOM,
    HANDLE_HALF_WIDTH,
    HANDLE_TOP,
    PALLET_TOP,
    TOTE_HEIGHT,
)

PROP_DIR = Path(__file__).resolve().parents[1] / "assets" / "props"

TOTE_HALF_H = TOTE_HEIGHT / 2
# Rolled lip top; the depth camera reads stack height off this surface.
RIM_TOP = TOTE_HALF_H
# Widest point of the tote: the rolled lip over the short end walls.
RIM_HALF_Y = 0.25

# Hook tool, in the tool frame of poc_sim.gripper: +X inboard, +Z up.
HOOK_PRONG_X = 0.090
HOOK_PRONG_Z = -0.1955
HOOK_NOSE_X = 0.126


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


def tote_shell():
    mesh = Mesh()
    base = -TOTE_HALF_H
    levels = [base, base + 0.006, HANDLE_BOTTOM, HANDLE_TOP, TOTE_HALF_H]
    span = levels[-1] - base
    n = len(rounded_rectangle(0.17, 0.22, 0.016, 0))
    for inner in [False, True]:
        for z in levels:
            f = (z - base) / span
            hx = 0.17 + 0.020 * f
            hy = 0.22 + 0.020 * f
            r = 0.016 + 0.004 * f
            if inner:
                hx -= 0.0025
                hy -= 0.0025
                r -= 0.0025
            mesh.vertices.extend(rounded_rectangle(hx, hy, r, z))
    layers = len(levels)
    offset = layers * n
    occupied = {}
    for k in range(layers - 1):
        for j in range(n):
            v = np.mean(
                [mesh.vertices[k * n + j], mesh.vertices[k * n + (j + 1) % n]], axis=0
            )
            # Hand openings in both short end walls, below the yellow bars; the
            # band between levels 2 and 3 is exactly the hole the prong enters.
            hole = k == 2 and abs(v[1]) > 0.225 and abs(v[0]) < HANDLE_HALF_WIDTH
            occupied[k, j] = not hole
            if hole:
                continue
            a = k * n + j
            b = k * n + (j + 1) % n
            c = (k + 1) * n + (j + 1) % n
            d = (k + 1) * n + j
            mesh.quad(a, b, c, d)
            mesh.quad(offset + d, offset + c, offset + b, offset + a)
    for (k, j), present in occupied.items():
        if not present:
            continue
        a = k * n + j
        b = k * n + (j + 1) % n
        c = (k + 1) * n + (j + 1) % n
        d = (k + 1) * n + j
        for neighbor, edge in [
            ((k - 1, j), (b, a)),
            ((k + 1, j), (d, c)),
            ((k, (j - 1) % n), (a, d)),
            ((k, (j + 1) % n), (c, b)),
        ]:
            if not occupied.get(neighbor, False):
                u, v = edge
                mesh.quad(u, v, offset + v, offset + u)
    mesh.rounded_slab(0.17, 0.22, 0.016, base, base + 0.006)
    return mesh


def hook_tool():
    """L-shaped handle hook: wrist plate, outboard column, inserted prong."""
    mesh = Mesh()
    z = HOOK_PRONG_Z
    mesh.loft(
        2,
        [
            (0.0, 0.010, 0.0, 0.055, 0.052),
            (-0.026, 0.010, 0.0, 0.055, 0.052),
            (-0.040, 0.019, 0.0, 0.044, 0.050),
            (-0.075, 0.0275, 0.0, 0.030, 0.048),
            (-0.140, 0.0275, 0.0, 0.020, 0.045),
            (z + 0.010, 0.0275, 0.0, 0.020, 0.043),
        ],
    )
    mesh.loft(
        0,
        [
            (0.0075, 0.0, z, 0.043, 0.0105),
            (0.055, 0.0, z, 0.042, 0.0100),
            (0.105, 0.0, z, 0.041, 0.0095),
            (HOOK_NOSE_X, 0.0, z + 0.0035, 0.032, 0.0055),
        ],
    )
    return mesh


def make_meshes():
    PROP_DIR.mkdir(parents=True, exist_ok=True)
    shell = tote_shell()
    rim = Mesh()
    # Upper rolled lip, lower reinforcing bead, and shoulder support flange.
    rim.ring(0.20, RIM_HALF_Y, 0.023, RIM_TOP - 0.009, 0.1905, 0.2405, 0.0135, 0.009)
    rim.ring(0.196, 0.246, 0.022, RIM_TOP - 0.034, 0.187, 0.237, 0.013, 0.007)
    rim.ring(0.195, 0.245, 0.022, RIM_TOP - 0.027, 0.187, 0.237, 0.014, 0.010)
    ribs = Mesh()
    # Shallow external moulding ribs, under the rim, inside the specified envelope.
    z = -0.01
    taper = 0.020 / (TOTE_HALF_H * 2)
    for sign in [-1, 1]:
        for y in [-0.195, -0.14, -0.085, 0.085, 0.14, 0.195]:
            x = sign * (0.17 + taper * (z + TOTE_HALF_H) + 0.001)
            ribs.slanted_box([x, y, z], [0.0014, 0.0024, 0.09], 0, sign * taper)
        for x in [-0.135, -0.08, 0.08, 0.135]:
            y = sign * (0.22 + taper * (z + TOTE_HALF_H) + 0.001)
            ribs.slanted_box([x, y, z], [0.0024, 0.0014, 0.09], 1, sign * taper)
    pallet = Mesh()
    # 1000 x 1200 x 123 mm; nine feet, three runners, open fork pockets.
    deck = PALLET_TOP - 0.023
    for x in [-0.43, 0, 0.43]:
        pallet.box([x, 0, 0.009], [0.07, 0.6, 0.009])
        for y in [-0.52, 0, 0.52]:
            pallet.box([x, y, (0.018 + deck) / 2], [0.07, 0.08, (deck - 0.018) / 2])
    for y in [-0.565, 0.565]:
        pallet.box([0, y, 0.009], [0.5, 0.035, 0.009])
    pallet.ring(0.5, 0.6, 0.018, deck, 0.472, 0.572, 0.006, 0.023)
    for x in [-0.3, -0.1, 0.1, 0.3]:
        pallet.box([x, 0, PALLET_TOP - 0.014], [0.012, 0.572, 0.014])
    for y in [-0.4, -0.2, 0, 0.2, 0.4]:
        pallet.box([0, y, PALLET_TOP - 0.014], [0.472, 0.012, 0.014])
    for x in np.linspace(-0.46, 0.46, 21):
        pallet.box([x, 0, PALLET_TOP - 0.007], [0.004, 0.572, 0.007])
    for y in np.linspace(-0.56, 0.56, 25):
        pallet.box([0, y, PALLET_TOP - 0.007], [0.472, 0.004, 0.007])
    return {
        name: mesh.write(name)
        for name, mesh in [
            ("tote_shell", shell),
            ("tote_rim", rim),
            ("tote_ribs", ribs),
            ("pallet_lattice", pallet),
            ("hook_tool", hook_tool()),
        ]
    }


def add_assets(asset):
    for name, path in make_meshes().items():
        E.SubElement(asset, "mesh", name=name, file=str(path))
    for name, rgba in [
        ("tote_plastic", ".075 .055 .27 1"),
        ("tote_edges", ".11 .08 .34 1"),
        ("tote_bar", ".98 .69 .035 1"),
        ("pallet_plastic", ".035 .26 .13 1"),
        ("tool_shell", ".085 .09 .105 1"),
        ("tool_metal", ".40 .43 .48 1"),
    ]:
        E.SubElement(
            asset,
            "material",
            name=name,
            rgba=rgba,
            specular=".32",
            shininess=".25",
            reflectance=".02",
        )


def hook_tool_geoms(side):
    """Visual shell of one printed handle hook; gripper.py owns its contact boxes."""
    geoms = [
        {"name": f"{side}_hook_shell", "type": "mesh", "mesh": "hook_tool",
         "material": "tool_shell"},
        {"name": f"{side}_hook_boss", "type": "cylinder", "pos": "0.01 0 -0.022",
         "size": "0.040 0.006", "material": "tool_metal"},
    ]
    # Bolt circle on the wrist plate, on the part the column does not cover.
    for k in range(10):
        a = 2 * math.pi * (k + 0.5) / 10
        geoms.append({
            "type": "cylinder",
            "pos": f"{0.01 + 0.030 * math.cos(a)} {0.030 * math.sin(a)} -0.0275",
            "size": "0.0045 0.0015",
            "rgba": ".03 .03 .035 1",
        })
    for x, y in [(-0.035, 0.034), (-0.035, -0.034), (0.057, 0.034), (0.057, -0.034)]:
        geoms.append({
            "type": "cylinder", "pos": f"{x} {y} -0.0275", "size": "0.005 0.0015",
            "rgba": ".03 .03 .035 1",
        })
    for g in geoms:
        g.update(contype="0", conaffinity="0", mass="0", group="1")
    return geoms


def visual_geom(parent, **kwargs):
    return E.SubElement(
        parent, "geom", contype="0", conaffinity="0", mass="0", group="1", **kwargs
    )


def add_tote_visual(body, index):
    for part, material in [
        ("shell", "tote_plastic"),
        ("rim", "tote_edges"),
        ("ribs", "tote_edges"),
    ]:
        visual_geom(
            body,
            name=f"tote_visual_{index}_{part}",
            type="mesh",
            mesh=f"tote_{part}",
            material=material,
        )
    for side in [-1, 1]:
        y = side * 0.245
        bar = round(RIM_TOP - 0.006, 4)
        visual_geom(
            body,
            type="capsule",
            fromto=f"-.173 {y} {bar} .173 {y} {bar}",
            size=".005",
            material="tote_bar",
        )
        for x in [-0.18, 0.18]:
            visual_geom(
                body,
                type="box",
                pos=f"{x} {y} {bar}",
                size=".009 .0045 .005",
                material="tote_bar",
            )
        for x in [-0.12, 0, 0.12]:
            visual_geom(
                body,
                type="box",
                pos=f"{x} {y} {bar}",
                size=".014 .0045 .005",
                material="tote_bar",
            )
    # Recessed label plaque and an abstract inventory barcode (not a vendor ID).
    visual_geom(
        body,
        type="box",
        pos="-.1845 -.105 .024",
        size=".001 .040 .025",
        rgba=".84 .85 .81 1",
    )
    for k in range(19):
        y = -0.140 + k * 0.0037
        visual_geom(
            body,
            type="box",
            pos=f"-.1856 {y} .024",
            size=f".0002 {(0.0005 if k % 3 else 0.0009)} .019",
            rgba=".04 .045 .05 1",
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
