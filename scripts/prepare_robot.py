"""Convert official collision URDF to MuJoCo; preserve original geometry/inertia.

The official collision meshes are trimesh convex decompositions: one OBJ holds
several `o` blocks, each an exactly convex piece. MuJoCo's OBJ loader reads only
the first block, so a single <mesh> would silently drop the rest — up to 82% of
the faces. Each piece is therefore written out as its own OBJ and attached as a
separate <collision>, which is also what MuJoCo wants: it hulls every mesh geom,
and hulling the pieces separately is what preserves the concave shape.
"""

import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / ".deps/dexmate_urdf/robots/humanoid/vega_1/vega_1.urdf"
PARTS = ROOT / "assets/collision"


def read_obj(path):
    """-> (vertices, [(name, [triangle, ...]), ...]); polygons are triangulated."""
    verts, parts, cur = [], [], None
    with open(path) as f:
        for line in f:
            if line.startswith("v "):
                verts.append(line.split()[1:4])
            elif line[:2] in ("o ", "g "):
                cur = (line.split(maxsplit=1)[1].strip(), [])
                parts.append(cur)
            elif line.startswith("f "):
                idx = [int(t.split("/")[0]) for t in line.split()[1:]]
                if cur is None:
                    cur = ("geometry_0", [])
                    parts.append(cur)
                # Negative indices are relative to the end of the vertex list.
                idx = [i if i > 0 else len(verts) + 1 + i for i in idx]
                for k in range(1, len(idx) - 1):
                    cur[1].append((idx[0], idx[k], idx[k + 1]))
    return verts, parts


def write_part(path, verts, tris):
    """One convex piece as a standalone OBJ, with vertices re-indexed."""
    used = sorted({i for t in tris for i in t})
    remap = {old: new for new, old in enumerate(used, start=1)}
    lines = [f"# split from {path.stem} by scripts/prepare_robot.py"]
    lines += [f"v {' '.join(verts[i - 1])}" for i in used]
    lines += [f"f {remap[a]} {remap[b]} {remap[c]}" for a, b, c in tris]
    path.write_text("\n".join(lines) + "\n")


def split_meshes(root):
    """Replace every multi-piece <collision> mesh with one <collision> per piece."""
    if PARTS.exists():
        shutil.rmtree(PARTS)
    PARTS.mkdir(parents=True)
    split, total = 0, 0
    for link in root.findall("link"):
        for col in list(link.findall("collision")):
            mesh = col.find("geometry/mesh")
            if mesh is None:
                continue
            src = (SOURCE.parent / mesh.get("filename")).resolve()
            if src.suffix.lower() != ".obj":
                continue
            total += 1
            verts, parts = read_obj(src)
            if len(parts) <= 1:
                continue
            index = list(link).index(col)
            link.remove(col)
            for i, (_, tris) in enumerate(parts):
                out = PARTS / f"{src.stem}_p{i}.obj"
                if not out.exists():            # shared meshes: write once
                    write_part(out, verts, tris)
                clone = ET.fromstring(ET.tostring(col))
                clone.find("geometry/mesh").set("filename", str(out))
                link.insert(index + i, clone)
            split += 1
            print(f"  split {src.name}: {len(parts)} pieces, "
                  f"{sum(len(t) for _, t in parts)} triangles")
    print(f"collision meshes: {total} total, {split} split into pieces")


def prepare():
    tree = ET.parse(SOURCE)
    root = tree.getroot()
    for link in root.findall("link"):
        for visual in link.findall("visual"):
            link.remove(visual)
    split_meshes(root)
    for mesh in root.findall(".//mesh"):
        mesh.set("filename", str((SOURCE.parent / mesh.get("filename")).resolve()))
    mj = ET.SubElement(root, "mujoco")
    ET.SubElement(
        mj, "compiler", discardvisual="true", fusestatic="false", balanceinertia="true"
    )
    path = ROOT / "assets/vega_collision.urdf"
    tree.write(path)
    model = mujoco.MjModel.from_xml_path(str(path))
    mujoco.mj_saveLastXML(str(ROOT / "assets/vega.xml"), model)
    print("Converted official Vega-1:", model.nq, "joints,", model.ngeom, "geoms")


if __name__ == "__main__":
    prepare()
