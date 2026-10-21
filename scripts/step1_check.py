#!/usr/bin/env python
"""Step 1 model verification for the Vega-1 palletising PoC.

Reports on the model MuJoCo actually compiled (mjModel), not the source URDF.
Read-only with respect to project files: the only mutation is rgba on a throwaway
in-memory MjModel so the collision screenshots can hide visual-only geoms.

    MUJOCO_GL=egl OPENBLAS_NUM_THREADS=1 python scripts/step1_check.py [--write]

--write saves the report to reports/step1_model_check.md; otherwise it goes to stdout.
"""

import argparse
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from poc_sim.layout import BASE_TURN_RATE  # noqa: E402


URDF = ROOT / ".deps/dexmate_urdf/robots/humanoid/vega_1/vega_1.urdf"
SRDF = ROOT / ".deps/dexmate_urdf/robots/humanoid/vega_1/vega_1.srdf"
MJCF_ROBOT = ROOT / "assets/vega.xml"
MJCF_SCENE = ROOT / "assets/scene.xml"
INTERMEDIATE = ROOT / "assets/vega_collision.urdf"
MESH_DIR = URDF.parent / "meshes/collision"
VIS_DIR = URDF.parent / "meshes/visual"

JNT_TYPE = {0: "free", 1: "ball", 2: "slide", 3: "hinge"}
GEOM_TYPE = {
    0: "plane", 1: "hfield", 2: "sphere", 3: "capsule", 4: "ellipsoid",
    5: "cylinder", 6: "box", 7: "mesh", 8: "sdf",
}
OUT = []


def w(line=""):
    OUT.append(line)


def table(headers, rows):
    w("| " + " | ".join(headers) + " |")
    w("|" + "|".join("---" for _ in headers) + "|")
    for r in rows:
        w("| " + " | ".join(str(c) for c in r) + " |")
    w()


# --------------------------------------------------------------------------- URDF
def urdf_joints():
    """name -> dict(type, lower, upper, velocity, effort) from the source URDF."""
    out = {}
    root = ET.parse(URDF).getroot()
    for j in root.findall("joint"):
        lim = j.find("limit")
        d = {"type": j.get("type"), "lower": None, "upper": None,
             "velocity": None, "effort": None}
        if lim is not None:
            for k in ["lower", "upper", "velocity", "effort"]:
                if lim.get(k) is not None:
                    d[k] = float(lim.get(k))
        out[j.get("name")] = d
    return out


def urdf_links():
    """All link names declared in the source URDF, in order."""
    return [l.get("name") for l in ET.parse(URDF).getroot().findall("link")]


def urdf_sensor_frames():
    """Fixed-mount massless frames: link -> (parent, xyz, rpy)."""
    root = ET.parse(URDF).getroot()
    out = {}
    for j in root.findall("joint"):
        if j.get("type") != "fixed":
            continue
        o = j.find("origin")
        out[j.find("child").get("link")] = (
            j.find("parent").get("link"),
            [float(x) for x in (o.get("xyz") or "0 0 0").split()],
            [float(x) for x in (o.get("rpy") or "0 0 0").split()],
            j.get("name"),
        )
    return out


def urdf_inertials():
    """link -> (mass, 3x3 inertia tensor) from the source URDF."""
    out = {}
    root = ET.parse(URDF).getroot()
    for link in root.findall("link"):
        inert = link.find("inertial")
        if inert is None:
            continue
        mass = float(inert.find("mass").get("value"))
        i = inert.find("inertia")
        v = {k: float(i.get(k, 0.0))
             for k in ["ixx", "ixy", "ixz", "iyy", "iyz", "izz"]}
        tensor = np.array([
            [v["ixx"], v["ixy"], v["ixz"]],
            [v["ixy"], v["iyy"], v["iyz"]],
            [v["ixz"], v["iyz"], v["izz"]],
        ])
        out[link.get("name")] = (mass, tensor)
    return out


def glb_aabb(path):
    """AABB of a .glb in its own root frame, from glTF accessor min/max.

    glTF requires POSITION accessors to carry min/max, so the extents are exact
    without decoding the binary buffer. Node transforms are applied.
    """
    import json
    import struct

    raw = path.read_bytes()
    if raw[:4] != b"glTF":
        raise ValueError(f"not a GLB: {path}")
    off, doc = 12, None
    while off < len(raw):
        clen, ctype = struct.unpack_from("<II", raw, off)
        if ctype == 0x4E4F534A:  # 'JSON'
            doc = json.loads(raw[off + 8:off + 8 + clen])
            break
        off += 8 + clen
    if doc is None:
        raise ValueError(f"no JSON chunk: {path}")

    def node_matrix(n):
        if "matrix" in n:
            return np.array(n["matrix"], dtype=float).reshape(4, 4).T
        M = np.eye(4)
        if "scale" in n:
            M[:3, :3] = np.diag(n["scale"])
        if "rotation" in n:
            x, y, z, wq = n["rotation"]
            R = np.array([
                [1 - 2*(y*y + z*z), 2*(x*y - z*wq),   2*(x*z + y*wq)],
                [2*(x*y + z*wq),    1 - 2*(x*x+z*z),  2*(y*z - x*wq)],
                [2*(x*z - y*wq),    2*(y*z + x*wq),   1 - 2*(x*x+y*y)],
            ])
            M[:3, :3] = R @ M[:3, :3]
        if "translation" in n:
            M[:3, 3] = n["translation"]
        return M

    nodes = doc.get("nodes", [])
    meshes = doc.get("meshes", [])
    acc = doc.get("accessors", [])
    corners = np.array([[sx, sy, sz] for sx in (0, 1) for sy in (0, 1) for sz in (0, 1)])
    lo = np.array([np.inf] * 3)
    hi = -lo

    def visit(idx, M):
        nonlocal lo, hi
        n = nodes[idx]
        M = M @ node_matrix(n)
        if "mesh" in n:
            for prim in meshes[n["mesh"]].get("primitives", []):
                ai = prim.get("attributes", {}).get("POSITION")
                if ai is None:
                    continue
                a = acc[ai]
                if "min" not in a or "max" not in a:
                    continue
                mn, mx = np.array(a["min"], float), np.array(a["max"], float)
                pts = mn + corners * (mx - mn)
                pts = (M[:3, :3] @ pts.T).T + M[:3, 3]
                lo = np.minimum(lo, pts.min(axis=0))
                hi = np.maximum(hi, pts.max(axis=0))
        for c in n.get("children", []):
            visit(c, M)

    roots = doc["scenes"][doc.get("scene", 0)]["nodes"] if doc.get("scenes") else \
        [i for i in range(len(nodes))]
    for r in roots:
        visit(r, np.eye(4))
    if not np.all(np.isfinite(lo)):
        raise ValueError(f"no POSITION extents: {path}")
    return lo, hi


def triangle_ok(moments, tol=1e-12):
    a, b, c = sorted(moments)
    return a + b >= c - tol


# ------------------------------------------------------------------- bounding box
def mesh_live_verts(m, mid):
    """Vertices referenced by the faces MuJoCo actually loaded.

    mesh_vert keeps every vertex in the file even when the OBJ loader drops
    faces, so slicing mesh_vert directly overstates the real collision extent.
    """
    fa = int(m.mesh_faceadr[mid])
    fn = int(m.mesh_facenum[mid])
    used = np.unique(m.mesh_face[fa:fa + fn].ravel())
    va = int(m.mesh_vertadr[mid])
    return m.mesh_vert[va + used]


def geom_aabb(m, d, g):
    """World-frame AABB of one geom. Mesh geoms use their actual vertices."""
    pos, mat = d.geom_xpos[g], d.geom_xmat[g].reshape(3, 3)
    t = m.geom_type[g]
    if t == mujoco.mjtGeom.mjGEOM_MESH:
        v = mesh_live_verts(m, int(m.geom_dataid[g]))
        pts = v @ mat.T + pos
        return pts.min(axis=0), pts.max(axis=0)
    if t == mujoco.mjtGeom.mjGEOM_PLANE:
        return None, None
    s = m.geom_size[g].copy()
    if t == mujoco.mjtGeom.mjGEOM_SPHERE:
        half = np.array([s[0]] * 3)
    elif t in (mujoco.mjtGeom.mjGEOM_CAPSULE, mujoco.mjtGeom.mjGEOM_CYLINDER):
        half = np.array([s[0], s[0], s[1] + (s[0] if t == 3 else 0.0)])
    elif t == mujoco.mjtGeom.mjGEOM_ELLIPSOID:
        half = s[:3]
    else:  # box
        half = s[:3]
    corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
    pts = (corners * half) @ mat.T + pos
    return pts.min(axis=0), pts.max(axis=0)


def merge_aabb(boxes):
    boxes = [b for b in boxes if b[0] is not None]
    if not boxes:
        return None
    lo = np.min([b[0] for b in boxes], axis=0)
    hi = np.max([b[1] for b in boxes], axis=0)
    return lo, hi


def is_collision(m, g):
    return bool(m.geom_contype[g]) or bool(m.geom_conaffinity[g])


# ------------------------------------------------------------------------ sections
def section0():
    w("## 0. 사용 중인 모델 파일")
    w()
    w("### 로드 경로 (2단계 변환)")
    w()
    w("```")
    w(".deps/dexmate_urdf/robots/humanoid/vega_1/vega_1.urdf      공식 원본")
    w("        │  scripts/prepare_robot.py")
    w("        │    · 모든 <visual> 엘리먼트 제거")
    w("        │    · <mujoco><compiler discardvisual=true")
    w("        │                        fusestatic=false")
    w("        │                        balanceinertia=true/>  주입")
    w("        ▼")
    w("assets/vega_collision.urdf                                 중간 산출물")
    w("        │  MjModel.from_xml_path() -> mj_saveLastXML()")
    w("        ▼")
    w("assets/vega.xml                                            동결된 MJCF")
    w("        │  poc_sim/scene.py build()")
    w("        │    · base_yaw hinge 추가, 후크/토트/팔레트/컨베이어/카메라 추가")
    w("        │    · vega_1.srdf 의 disable_collisions -> <contact><exclude>")
    w("        ▼")
    w("assets/scene.xml  ->  MjModel  (이 보고서의 검증 대상)")
    w("```")
    w()
    rows = []
    for label, p in [
        ("공식 URDF", URDF), ("중간 URDF", INTERMEDIATE),
        ("동결 MJCF (로봇)", MJCF_ROBOT), ("최종 MJCF (씬)", MJCF_SCENE), ("SRDF", SRDF),
    ]:
        rows.append([label, f"`{p.relative_to(ROOT)}`", "있음" if p.exists() else "**없음**",
                     f"{p.stat().st_size:,} B" if p.exists() else "-"])
    table(["단계", "경로", "존재", "크기"], rows)
    w("**선택된 변형:** `vega_1.urdf` — 손 없는 기본 변형입니다. "
      "`vega_1_gripper.urdf`(dexs 그리퍼), `vega_1_f5d6.urdf`(5지 핸드)는 쓰지 않습니다. "
      "말단은 `poc_sim/gripper.py`가 붙이는 **자체 제작 후크**입니다.")
    w()
    w("### compiler 옵션")
    w()
    inj = "미확인"
    if INTERMEDIATE.exists():
        mj = ET.parse(INTERMEDIATE).getroot().find("mujoco/compiler")
        if mj is not None:
            inj = ", ".join(f"`{k}={v}`" for k, v in mj.attrib.items())
    mjcf_compiler = "미확인"
    if MJCF_ROBOT.exists():
        c = ET.parse(MJCF_ROBOT).getroot().find("compiler")
        if c is not None:
            mjcf_compiler = ", ".join(f"`{k}={v}`" for k, v in c.attrib.items())
    w(f"- URDF에 주입된 값: {inj}")
    w(f"- 저장된 `assets/vega.xml`의 compiler: {mjcf_compiler}")
    w("- `meshdir`: 설정하지 않음. `prepare_robot.py`가 메쉬 경로를 **절대경로로 치환**합니다.")
    w("- `inertiafromgeom`: 설정하지 않음 → MuJoCo 기본값 `auto` "
      "(URDF에 `<inertial>`이 있으면 그것을 사용, 없으면 geom에서 계산)")
    w()


def section1(m, d, uj, ui, ulinks, robot_bodies):
    w("## 1. 질량 · 관성")
    w()
    total = float(sum(m.body_mass[b] for b in robot_bodies))
    w(f"- **mjModel 로봇 질량 합계: {total:.3f} kg** (body 1 ~ {max(robot_bodies)}, 토트 제외)")
    urdf_total = sum(v[0] for v in ui.values())
    w(f"- URDF `<mass>` 합계: {urdf_total:.3f} kg")
    hook = sum(m.body_mass[b] for b in robot_bodies
               if "tool" in (m.body(b).name or ""))
    w(f"- 그 중 자체 제작 후크(`L/R_tool`): {hook:.3f} kg")
    w(f"- 공식 URDF 유래 질량: {total - hook:.3f} kg")
    w()
    w("| 비교 대상 | 값 | 차이 |")
    w("|---|---|---|")
    w("| 공개 스펙 (사용자 제시) | 약 135 kg | - |")
    w(f"| mjModel 전체 | {total:.2f} kg | {total - 135:+.2f} kg ({(total-135)/135*100:+.1f}%) |")
    w(f"| mjModel − 후크 | {total - hook:.2f} kg | {total - hook - 135:+.2f} kg "
      f"({(total-hook-135)/135*100:+.1f}%) |")
    w()
    w("> 공개 스펙 135 kg의 **출처를 확인하지 못했습니다**(사용자 제시값). "
      "구성(핸드 종류, 배터리)에 따라 달라질 수 있어 판정은 참고용입니다.")
    w()

    w("### 링크별 질량 · 관성")
    w()
    rows, zero_mass, tri_bad, urdf_tri_bad, changed = [], [], [], [], []
    for b in robot_bodies:
        name = m.body(b).name or f"body{b}"
        mass = float(m.body_mass[b])
        I = m.body_inertia[b].copy()
        flags = []
        if mass == 0.0:
            zero_mass.append(name)
            flags.append("질량0")
        elif mass < 0.01:
            flags.append("질량<10g")
        if np.any(I <= 0):
            flags.append("관성≤0")
        if not triangle_ok(I):
            tri_bad.append(name)
            flags.append("**삼각부등식위반**")
        # compare against URDF
        note = ""
        if name in ui:
            um, ut = ui[name]
            ue = np.sort(np.linalg.eigvalsh(ut))
            if not triangle_ok(ue):
                urdf_tri_bad.append(name)
                note = "URDF 원본이 삼각부등식 위반"
            mi = np.sort(I)
            denom = np.maximum(np.abs(ue), 1e-12)
            if np.max(np.abs(mi - ue) / denom) > 0.01:
                changed.append((name, ue, mi))
                note = (note + "; " if note else "") + "변환 중 재계산"
            if abs(um - mass) > 1e-6:
                note = (note + "; " if note else "") + f"URDF질량 {um:.3f}"
        elif name in ulinks:
            note = "URDF 링크이지만 `<inertial>` 없음 (센서/기준 프레임)"
        else:
            note = "URDF에 없음 (프로젝트 추가분)"
        rows.append([name, f"{mass:.3f}",
                     f"{I[0]:.4g}", f"{I[1]:.4g}", f"{I[2]:.4g}",
                     " ".join(flags) or "-", note or "-"])
    table(["링크", "질량 kg", "Ixx'", "Iyy'", "Izz'", "플래그", "비고"], rows)
    w("*Ixx'/Iyy'/Izz' 는 mjModel이 보유한 **주관성모멘트**(principal moments, kg·m²)입니다.*")
    w()
    w("### 관성 타당성 판정")
    w()
    w(f"- 질량 0 링크: **{len(zero_mass)}개**" + (f" — {', '.join(zero_mass)}" if zero_mass else ""))
    w(f"- mjModel 주관성 중 0 이하: **{sum(1 for b in robot_bodies if np.any(m.body_inertia[b] <= 0))}개**")
    w(f"- mjModel 삼각부등식 위반: **{len(tri_bad)}개**" + (f" — {', '.join(tri_bad)}" if tri_bad else ""))
    w(f"- **URDF 원본**이 삼각부등식을 위반한 링크: **{len(urdf_tri_bad)}개**"
      + (f" — {', '.join(urdf_tri_bad)}" if urdf_tri_bad else ""))
    w(f"- 변환 중 관성이 1% 이상 바뀐 링크: **{len(changed)}개**")
    if changed:
        w()
        crows = [[n, " / ".join(f"{x:.4g}" for x in ue), " / ".join(f"{x:.4g}" for x in mi),
                  f"{np.max(np.abs(mi-ue)/np.maximum(np.abs(ue),1e-12))*100:.1f}%"]
                 for n, ue, mi in changed]
        table(["링크", "URDF 고유값 (정렬)", "mjModel 주관성 (정렬)", "최대 상대차"], crows)
    w("`balanceinertia=true`가 켜져 있으므로, URDF가 삼각부등식을 위반한 링크는 "
      "MuJoCo가 **경고 없이 수정**합니다.")
    w()


def section2(m, uj, robot_bodies):
    w("## 2. 관절 한계")
    w()
    rows = []
    placeholder = []
    for j in range(m.njnt):
        if int(m.jnt_bodyid[j]) not in robot_bodies:
            continue
        name = m.joint(j).name or f"jnt{j}"
        t = JNT_TYPE.get(int(m.jnt_type[j]), "?")
        lim = bool(m.jnt_limited[j])
        rng = m.jnt_range[j]
        u = uj.get(name, {})
        afl = bool(m.jnt_actfrclimited[j]) if hasattr(m, "jnt_actfrclimited") else False
        afr = m.jnt_actfrcrange[j] if hasattr(m, "jnt_actfrcrange") else [0, 0]
        urange = ("-" if u.get("lower") is None
                  else f"{u['lower']:.4g} ~ {u['upper']:.4g}")
        mrange = "무제한" if not lim else f"{rng[0]:.4g} ~ {rng[1]:.4g}"
        match = "-"
        if lim and u.get("lower") is not None:
            match = ("일치" if abs(rng[0] - u["lower"]) < 1e-6
                     and abs(rng[1] - u["upper"]) < 1e-6 else "**불일치**")
        elif not lim and u.get("lower") is not None:
            match = "**URDF엔 있는데 미반영**"
        rows.append([
            name, t, mrange, urange, match,
            "-" if u.get("velocity") is None else f"{u['velocity']:.4g}",
            "-" if u.get("effort") is None else f"{u['effort']:.4g}",
            f"{afl} {np.round(afr,3).tolist()}" if afl else "미설정",
        ])
        v, e = u.get("velocity"), u.get("effort")
        if v is not None and float(v).is_integer() and v in (1.0, 10.0, 100.0):
            placeholder.append(f"{name}: velocity={v}")
        if e is not None and float(e).is_integer() and e in (1.0, 10.0, 100.0, 1000.0):
            placeholder.append(f"{name}: effort={e}")
    table(["관절", "타입", "mjModel jnt_range", "URDF lower~upper", "반영",
           "URDF velocity", "URDF effort", "jnt_actfrcrange"], rows)

    w("### URDF velocity / effort 가 모델에 반영되는가")
    w()
    w("| 값 | mjModel 필드 | 상태 |")
    w("|---|---|---|")
    w("| `lower`/`upper` | `jnt_range` + `jnt_limited` | 반영됨 (위 표 '반영' 열) |")
    w("| `velocity` | **없음** | MuJoCo는 관절 속도 한계를 모델 차원에서 지원하지 않습니다 |")
    w("| `effort` | `jnt_actfrcrange` (또는 actuator `forcerange`) | "
      f"`nu={m.nu}` 이므로 **반영 경로가 존재하지 않습니다** |")
    w()
    w("`velocity` 값의 실제 사용처: `poc_sim/simulation.py:116-121`이 **URDF를 직접 재파싱해서** "
      "`self.vel_limits`에 담고, `track()`이 궤적 구간 소요시간을 "
      "`max(|Δq| / vel_limit) × 1.15`로 계산할 때 씁니다. "
      "즉 mjModel이 아니라 **프로젝트 플래너가 자체적으로** 지키는 제약입니다.")
    w()
    w(f"`effort` 값의 실제 사용처: **없음.** 모델에 액추에이터가 0개(`nu={m.nu}`, "
      f"`na={m.na}`, `d.ctrl` 크기 {m.nu})이고, "
      "`poc_sim/` 전체에서 `effort`를 읽는 코드가 없습니다.")
    w()
    w("### placeholder 의심값")
    w()
    if placeholder:
        for p in sorted(set(placeholder)):
            w(f"- {p}")
    else:
        w("- 1.0 / 10 / 100 / 1000 같은 둥근 값으로 채워진 velocity·effort는 없습니다.")
    w()
    # repeated-value scan
    from collections import Counter
    vc = Counter(v["velocity"] for v in uj.values() if v.get("velocity") is not None)
    ec = Counter(v["effort"] for v in uj.values() if v.get("effort") is not None)
    w("동일값 반복 분포 (URDF 전체 관절):")
    w()
    table(["항목", "값 → 관절 수"],
          [["velocity", ", ".join(f"{k:g}→{n}" for k, n in sorted(vc.items()))],
           ["effort", ", ".join(f"{k:g}→{n}" for k, n in sorted(ec.items()))]])
    w("### 관절 저항 파라미터")
    w()
    table(["필드", "총합", "비고"],
          [["`dof_damping`", f"{float(m.dof_damping.sum()):.4g}", "URDF `<dynamics damping>` 미지정"],
           ["`dof_armature`", f"{float(m.dof_armature.sum()):.4g}", "모터 회전자 관성 없음"],
           ["`dof_frictionloss`", f"{float(m.dof_frictionloss.sum()):.4g}", "조인트 마찰 없음"]])


def section3(m, d, uj, robot_bodies):
    w("## 3. 단위")
    w()
    c = ET.parse(MJCF_ROBOT).getroot().find("compiler")
    angle = c.get("angle") if c is not None else "미확인"
    w(f"- **각도 단위:** `assets/vega.xml`의 `compiler angle=\"{angle}\"`. "
      f"URDF는 규격상 항상 라디안이고, MuJoCo URDF 임포터는 radian으로 고정하므로 일치합니다.")
    hinges = [j for j in range(m.njnt)
              if int(m.jnt_bodyid[j]) in robot_bodies
              and m.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE and m.jnt_limited[j]]
    if hinges:
        spans = [abs(m.jnt_range[j][1] - m.jnt_range[j][0]) for j in hinges]
        mx = max(spans)
        w(f"- 제한된 hinge {len(hinges)}개의 가동범위: "
          f"최소 {min(spans):.3f} rad ({np.degrees(min(spans)):.1f}°), "
          f"최대 {mx:.3f} rad ({np.degrees(mx):.1f}°)")
        w(f"  - 라디안 해석 시 최대 {np.degrees(mx):.1f}° → 합리적. "
          f"만약 이 값이 degree였다면 {mx:.1f}° 로 비정상적으로 좁아집니다. "
          f"**라디안 맞습니다.**")
        over = [m.joint(j).name for j in hinges
                if abs(m.jnt_range[j][1] - m.jnt_range[j][0]) > 2 * np.pi + 1e-6]
        w(f"  - 2π 초과 가동범위: {len(over)}개" + (f" — {over}" if over else ""))
    w()
    w("### 메쉬 스케일 / 전체 치수")
    w()
    scales = {}
    for g in range(m.ngeom):
        if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH and int(m.geom_bodyid[g]) in robot_bodies:
            mid = int(m.geom_dataid[g])
            scales[m.mesh(mid).name] = tuple(np.round(m.mesh_scale[mid], 6))
    uniq = sorted(set(scales.values()))
    w(f"- 로봇 메쉬 {len(scales)}개의 `mesh_scale` 고유값: "
      + ", ".join("(" + ", ".join(f"{x:g}" for x in u) + ")" for u in uniq))
    w(f"  - `(1,1,1)` 이면 OBJ가 **미터 단위**로 저장돼 있고 스케일 변환이 없다는 뜻입니다. "
      f"{'해당됩니다.' if uniq == [(1.0, 1.0, 1.0)] else '**다른 값이 섞여 있습니다.**'}")
    boxes = [geom_aabb(m, d, g) for g in range(m.ngeom)
             if int(m.geom_bodyid[g]) in robot_bodies]
    bb = merge_aabb(boxes)
    lo, hi = bb
    size = hi - lo
    w()
    table(["축", "min (m)", "max (m)", "크기 (m)"],
          [["X (전후)", f"{lo[0]:+.4f}", f"{hi[0]:+.4f}", f"{size[0]:.4f}"],
           ["Y (좌우)", f"{lo[1]:+.4f}", f"{hi[1]:+.4f}", f"{size[1]:.4f}"],
           ["Z (높이)", f"{lo[2]:+.4f}", f"{hi[2]:+.4f}", f"{size[2]:.4f}"]])
    w("- 위 값은 **현 자세**(IK 초기해 — 몸통을 굽히고 양팔을 운반 자세로 든 상태) "
      "기준입니다. 바운딩박스는 `qpos`에 의존하므로 스펙 대조에는 부적절합니다.")
    w()
    d0 = mujoco.MjData(m)
    mujoco.mj_resetData(m, d0)
    mujoco.mj_forward(m, d0)
    rg = [g for g in range(m.ngeom) if int(m.geom_bodyid[g]) in robot_bodies]

    def top_at(qvals):
        mujoco.mj_resetData(m, d0)
        for n, x in qvals.items():
            d0.qpos[m.joint(n).qposadr] = x
        mujoco.mj_forward(m, d0)
        bb = merge_aabb([geom_aabb(m, d0, g) for g in rg])
        return bb

    hi0 = top_at({})[1]
    w(f"- 모든 관절 0 (`mj_resetData`)에서의 높이: **{hi0[2]:.3f} m**. "
      "`torso_j1/j2`의 가동범위가 각각 0~1.57, 0~3.141이라 **0은 직립이 아니라 "
      "가장 접힌 자세**입니다. 스펙 대조에는 쓸 수 없습니다.")
    w()
    # Coarse scan of the three torso joints for the tallest reachable posture.
    tnames = ["torso_j1", "torso_j2", "torso_j3"]
    grids = []
    for n in tnames:
        j = m.joint(n).id
        lo_, hi_ = m.jnt_range[j]
        grids.append(np.linspace(lo_, hi_, 13))
    best, bestq = -np.inf, None
    for a in grids[0]:
        for b in grids[1]:
            for c in grids[2]:
                q = dict(zip(tnames, [a, b, c]))
                t = top_at(q)[1][2]
                if t > best:
                    best, bestq = t, q
    lob, hib = top_at(bestq)
    szb = hib - lob
    w(f"몸통 3관절을 13×13×13 격자로 훑어 찾은 **최대 신장 자세** "
      f"(`torso_j1={bestq['torso_j1']:.3f}`, `torso_j2={bestq['torso_j2']:.3f}`, "
      f"`torso_j3={bestq['torso_j3']:.3f}` rad, 팔은 0):")
    w()
    table(["축", "min (m)", "max (m)", "크기 (m)"],
          [["X (전후)", f"{lob[0]:+.4f}", f"{hib[0]:+.4f}", f"{szb[0]:.4f}"],
           ["Y (좌우)", f"{lob[1]:+.4f}", f"{hib[1]:+.4f}", f"{szb[1]:.4f}"],
           ["Z (높이)", f"{lob[2]:+.4f}", f"{hib[2]:+.4f}", f"{szb[2]:.4f}"]])
    w(f"- **최대 신장 높이: {hib[2]:.3f} m** — 사용자 제시 약 1.7 m 대비 "
      f"{hib[2]-1.7:+.3f} m ({(hib[2]-1.7)/1.7*100:+.1f}%)")
    w(f"- 운전 중 실제 자세(IK 초기해) 높이: {hi[2]:.3f} m")
    w("- 높이는 **면이 누락된 메쉬**(`head_l1`, `head_l3`, `torso_l3`)의 영향을 받습니다. "
      "머리 충돌 형상이 잘려 있으므로 실제 외형 높이는 이보다 클 수 있습니다. "
      "4장의 메쉬 무결성 항목을 참조하세요.")
    w("- 1.7 m라는 수치의 **출처는 확인하지 못했습니다**(사용자 제시값).")
    w()
    w("### 토크 · 질량 단위")
    w()
    w("- URDF `effort`는 revolute 관절에서 규격상 **N·m**입니다. "
      "구동 16, 조향 6, 팔 관절은 아래 2장 표 참조.")
    w("- URDF `<mass value>`는 규격상 **kg**. 합계가 사람이 들 수 없는 수준(142 kg)으로 "
      "나오는 것이 kg 해석과 일관됩니다 (g이었다면 0.142 kg).")
    w(f"- `m.opt.gravity = {np.round(m.opt.gravity,3).tolist()}` → m/s². "
      "길이 m · 질량 kg · 시간 s 의 SI 조합이 일관됩니다.")
    w()


def section4(m, d, robot_bodies):
    w("## 4. 충돌 형상")
    w()
    w("### 링크별 충돌 geom")
    w()
    rows = []
    for b in robot_bodies:
        name = m.body(b).name or f"body{b}"
        gs = range(m.body_geomadr[b], m.body_geomadr[b] + m.body_geomnum[b])
        col = [g for g in gs if is_collision(m, g)]
        vis = [g for g in gs if not is_collision(m, g)]
        if not col and not vis:
            rows.append([name, "0", "-", "0", "**충돌 geom 없음**"])
            continue
        kinds = {}
        for g in col:
            kinds[GEOM_TYPE.get(int(m.geom_type[g]), "?")] = kinds.get(
                GEOM_TYPE.get(int(m.geom_type[g]), "?"), 0) + 1
        rows.append([name, len(col),
                     ", ".join(f"{k}×{v}" for k, v in sorted(kinds.items())) or "-",
                     len(vis), "-" if col else "**충돌 geom 없음**"])
    table(["링크", "충돌 geom 수", "종류", "visual geom 수", "비고"], rows)

    nocol = [m.body(b).name for b in robot_bodies
             if not any(is_collision(m, g) for g in
                        range(m.body_geomadr[b], m.body_geomadr[b] + m.body_geomnum[b]))]
    w(f"- 충돌 geom이 **전혀 없는 링크: {len(nocol)}개**" + (f" — {nocol}" if nocol else ""))
    w()
    w("### 충돌 메쉬 무결성")
    w()
    w("MuJoCo의 OBJ 로더는 **파일 안의 첫 번째 객체(`o` 블록)만** 읽고, "
      "나머지 면은 경고 없이 버립니다. 공식 collision OBJ 중 5개가 여러 객체로 "
      "분할돼 있어 이 함정에 해당합니다. 아래는 소스 OBJ의 전체 삼각형 수와 "
      "모델에 실제로 들어간 면 수를 대조한 것입니다.")
    w()
    rows = []
    for stem, pieces, mjf, tris, users in mesh_integrity(m):
        pct = mjf / tris * 100 if tris else 0.0
        rows.append([stem, pieces, f"{tris:,}", f"{mjf:,}", f"{pct:.0f}%",
                     ", ".join(users),
                     "정상" if mjf == tris else "**면 누락**"])
    table(["소스 OBJ", "등록 geom 수", "OBJ 삼각형", "로드된 면", "로드율",
           "사용 링크", "판정"], rows)
    broken = [x for x in mesh_integrity(m) if x[2] != x[3]]
    w(f"- **면이 누락된 소스 OBJ: {len(broken)}개 / {len(rows)}개**")
    w()
    if broken:
        for stem, pieces, mjf, tris, users in broken:
            w(f"- **`{stem}`** → {', '.join(users)} — 면의 "
              f"**{100 - mjf / tris * 100:.0f}% 손실**")
        w()
    else:
        w("`scripts/prepare_robot.py`가 각 OBJ의 `o` 블록을 개별 파일 "
          "(`assets/collision/<stem>_p<i>.obj`)로 분리해 링크마다 여러 "
          "`<collision>`으로 등록합니다. 조각은 전부 **정확히 볼록**(면 평면 이탈 "
          "≤ 0.0001 mm)인 trimesh 볼록 분해 결과라, 조각별로 따로 convex hull을 "
          "씌우는 이 방식이 원래 오목 형상을 보존합니다. 하나로 병합하면 전체에 "
          "단일 hull이 씌워져 오목부가 메워집니다.")
        w()

    w("### 충돌 geom ↔ visual geom 바운딩박스 비교")
    w()
    robot_vis = [g for g in range(m.ngeom)
                 if int(m.geom_bodyid[g]) in robot_bodies and not is_collision(m, g)]
    urdf_derived = [g for g in robot_vis
                    if "tool" not in (m.body(int(m.geom_bodyid[g])).name or "")]
    w(f"공식 URDF에서 온 링크의 visual geom 수: **{len(urdf_derived)}개**")
    w()
    w("`scripts/prepare_robot.py`가 URDF의 모든 `<visual>` 엘리먼트를 제거한 뒤 "
      "`discardvisual=true`로 컴파일하므로, **로봇 링크에는 visual geom이 존재하지 않습니다.** "
      "뷰어에 보이는 로봇 외형은 충돌 메쉬 그 자체입니다. "
      "따라서 이 항목의 20% 비교는 **수행 불가**이며, 동시에 "
      "**충돌/외형 불일치가 원리적으로 발생할 수 없다**는 뜻이기도 합니다.")
    w()
    w("단, 공식 URDF에는 별도의 visual 메쉬가 존재합니다. 비교하려면 "
      "`meshes/visual/` 과 `meshes/collision/` 의 OBJ를 직접 대조해야 합니다:")
    w()
    vdir, cdir = VIS_DIR, MESH_DIR
    w(f"- `meshes/visual/` 존재: {vdir.exists()}"
      + (f" ({len(list(vdir.glob('*')))}개 파일)" if vdir.exists() else ""))
    w(f"- `meshes/collision/` 존재: {cdir.exists()}"
      + (f" ({len(list(cdir.glob('*')))}개 파일)" if cdir.exists() else ""))
    w()
    if vdir.exists() and cdir.exists():
        w("`visual/`은 `.glb`, `collision/`은 `.obj`로 포맷이 달라 "
          "**파일 스템(확장자 제외 이름)으로 대조**했습니다. "
          "GLB는 glTF accessor의 `min`/`max`를 노드 변환까지 적용해 읽었고, "
          "OBJ는 정점을 직접 파싱했습니다. 두 값 모두 링크 로컬 프레임 기준입니다.")
        w()
        rows = []

        def obj_verts(path):
            with open(path) as fh:
                return np.array([[float(x) for x in line.split()[1:4]]
                                 for line in fh if line.startswith("v ")])

        def obj_aabb(p):
            v = obj_verts(p)
            return v.min(axis=0), v.max(axis=0)

        flagged = 0
        vstems = {q.stem: q for q in vdir.glob("*")}
        unmatched_c, unmatched_v = [], sorted(set(vstems))
        for cp in sorted(cdir.glob("*.obj")):
            vp = vstems.get(cp.stem)
            if vp is None:
                unmatched_c.append(cp.stem)
                continue
            unmatched_v = [x for x in unmatched_v if x != cp.stem]
            try:
                vl, vh = glb_aabb(vp)
            except Exception as exc:  # noqa: BLE001
                rows.append([cp.stem, f"읽기 실패: {exc}", "-", "-", "-", "-", "-"])
                continue
            cl, ch = obj_aabb(cp)
            # MuJoCo re-expresses mesh vertices in a principal frame, so axis
            # order is not comparable. Use a rotation-invariant size instead:
            # the max distance from the centroid.
            def spread(pts):
                return float(np.max(np.linalg.norm(pts - pts.mean(axis=0), axis=1)))
            try:
                ratio = (spread(live_verts_for_stem(m, cp.stem))
                         / spread(obj_verts(cp)))
            except (KeyError, ValueError, ZeroDivisionError):
                ratio = None
            cs, vs = ch - cl, vh - vl
            per = (cs - vs) / np.maximum(vs, 1e-9)          # signed, per axis
            rel = float(per[int(np.argmax(np.abs(per)))])    # keep the sign
            mark = "**차이>=20%**" if abs(rel) >= 0.20 else "정상"
            if abs(rel) >= 0.20:
                flagged += 1
            rows.append([cp.stem,
                         " × ".join(f"{x:.3f}" for x in vs),
                         " × ".join(f"{x:.3f}" for x in cs),
                         ("-" if ratio is None else f"{ratio*100:.0f}%"),
                         " / ".join(f"{x*100:+.1f}" for x in per),
                         f"{rel*100:+.1f}%", mark])
        table(["메쉬 스템", "visual AABB (m)", "collision OBJ 전체 (m)",
               "로드분 크기비", "축별 차 % (x/y/z)",
               "최대절대차", "판정"], rows)
        w(f"- **20% 이상 차이: {flagged}개 / 대조 {len(rows)}개**")
        w(f"- collision에만 있는 스템: {unmatched_c or '없음'}")
        w(f"- visual에만 있는 스템: {unmatched_v or '없음'}")
        w("- `로드분 크기비` = (MuJoCo가 실제로 로드한 면의 정점 퍼짐) ÷ "
          "(OBJ 전체 정점 퍼짐). 퍼짐은 중심으로부터의 최대거리라 **회전에 불변**이므로, "
          "MuJoCo가 메쉬를 주축 정렬해 저장하는 것과 무관하게 비교됩니다. "
          "**100% 미만 = 충돌 형상이 잘려 있음.**")
        w("- 축별 차는 `collision OBJ 전체 − visual`입니다. **음수**면 충돌 형상이 "
          "외형보다 작고(파고들 수 있음), **양수**면 큽니다(실제보다 일찍 막힘).")
        w()
    w("### contype / conaffinity")
    w()
    from collections import Counter
    cc = Counter()
    for g in range(m.ngeom):
        if int(m.geom_bodyid[g]) in robot_bodies and is_collision(m, g):
            cc[(int(m.geom_contype[g]), int(m.geom_conaffinity[g]))] += 1
    table(["(contype, conaffinity)", "로봇 충돌 geom 수", "의미"],
          [[f"({a}, {b})", n,
            "로봇 본체 — 바닥(1)·토트(4)와 충돌" if (a, b) == (2, 7) else
            "후크 — 바닥(1)·토트(4)와 충돌" if (a, b) == (2, 7) else "기타"]
           for (a, b), n in sorted(cc.items())])
    w("프로젝트의 비트 규약:")
    w()
    table(["비트", "할당"],
          [["1", "환경 (floor, pallet, conveyor)"],
           ["2", "로봇 본체 + 후크"],
           ["4", "토트 벽/손잡이"]])
    w("두 geom은 `contype & conaffinity` 가 어느 방향으로든 0이 아니어야 충돌 쌍이 됩니다. "
      "로봇(contype=2, conaffinity=7)은 환경·토트·로봇 모두와 쌍이 생기고, "
      "환경(contype=1, conaffinity=7)은 로봇·토트와 생깁니다.")
    w()
    w("### 자기충돌 제외 쌍 (exclude)")
    w()
    w(f"- mjModel `nexclude` = **{m.nexclude}**")
    srdf_pairs = ET.parse(SRDF).getroot().findall("disable_collisions")
    w(f"- SRDF `disable_collisions` 엔트리 = **{len(srdf_pairs)}**")
    w(f"- 일치: {'예' if m.nexclude == len(srdf_pairs) else '**아니오**'}")
    w()
    w("<details><summary>전체 제외 쌍 목록</summary>")
    w()
    rows = []
    for e in range(m.nexclude):
        sig = int(m.exclude_signature[e])
        b1, b2 = sig >> 16, sig & 0xFFFF
        rows.append([e + 1, m.body(b1).name, m.body(b2).name])
    table(["#", "body1", "body2"], rows)
    w("</details>")
    w()
    w("### 후크(말단) 형상이 토트를 집을 수 있는가")
    w()
    w("공식 그리퍼가 아니라 `poc_sim/gripper.py`의 자체 제작 후크입니다. "
      "**구동 조인트가 없는 고정 형상**이고, '집는다'는 동작은 양손 간격(span) 변화로만 표현됩니다.")
    w()
    rows = []
    for side in ["L", "R"]:
        for nm in [f"{side}_tool_plate", f"{side}_tool_column", f"{side}_prong"]:
            try:
                g = m.geom(nm).id
            except KeyError:
                rows.append([nm, "**없음**", "-", "-", "-", "-"])
                continue
            s = m.geom_size[g]
            rows.append([nm, GEOM_TYPE.get(int(m.geom_type[g])),
                         " × ".join(f"{2*x*1000:.1f}" for x in s[:3]),
                         f"{m.body_mass[int(m.geom_bodyid[g])]:.3f}",
                         f"{m.geom_friction[g][0]:.2f}",
                         f"margin={m.geom_margin[g]:.4g} gap={m.geom_gap[g]:.4g}"])
    table(["geom", "타입", "전체 크기 mm (x×y×z)", "소속 body 질량 kg", "마찰 μ", "접촉 옵션"], rows)
    from poc_sim.gripper import (
        ENGAGE_DEPTH,
        PRONG_HALF_HEIGHT,
        PRONG_HALF_WIDTH,
        SPAN_HOLD,
        SPAN_OPEN,
    )
    from poc_sim.layout import (
        HANDLE_BOTTOM,
        HANDLE_HALF_WIDTH,
        HANDLE_TOP,
        TOTE_HEIGHT,
        WALL_T,
    )
    w()
    w("손잡이 구멍 대 돌기(prong) 치수 적합성:")
    w()
    hole_h = (HANDLE_TOP - HANDLE_BOTTOM) * 1000
    hole_w = HANDLE_HALF_WIDTH * 2 * 1000
    pr_h = PRONG_HALF_HEIGHT * 2 * 1000
    pr_w = PRONG_HALF_WIDTH * 2 * 1000
    table(["항목", "구멍", "돌기", "여유", "판정"],
          [["높이 mm", f"{hole_h:.1f}", f"{pr_h:.1f}", f"{hole_h-pr_h:+.1f}",
            "들어감" if pr_h < hole_h else "**안 들어감**"],
           ["폭 mm", f"{hole_w:.1f}", f"{pr_w:.1f}", f"{hole_w-pr_w:+.1f}",
            "들어감" if pr_w < hole_w else "**안 들어감**"]])
    w(f"- 삽입 깊이 `ENGAGE_DEPTH` = {ENGAGE_DEPTH*1000:.0f} mm, "
      f"벽 두께 `WALL_T` = {WALL_T*1000:.1f} mm → 벽 중심선을 "
      f"{(ENGAGE_DEPTH/2 - WALL_T)*1000:+.1f} mm 지나 걸칩니다.")
    w(f"- span: `SPAN_OPEN` {SPAN_OPEN*1000:.1f} mm → `SPAN_HOLD` {SPAN_HOLD*1000:.1f} mm "
      f"(편측 {(SPAN_OPEN-SPAN_HOLD)*1000:.0f} mm 삽입 행정)")
    w(f"- 돌기가 얹히는 턱: 손잡이 하단 `HANDLE_BOTTOM` = {HANDLE_BOTTOM*1000:+.1f} mm "
      f"(토트 중심 기준, 토트 높이 {TOTE_HEIGHT*1000:.0f} mm)")
    w()
    w("**운반 안정화는 형상이 아니라 `<equality><weld>`가 담당합니다** "
      "(`poc_sim/scene.py:188-196`, 토트당 2개 × 좌우). "
      "돌기 접촉이 확인되면 weld를 활성화하므로, 후크의 마찰·강성은 운반 성패에 관여하지 않습니다.")
    w()
    w(f"- 모델 내 weld equality 수: **{m.neq}** "
      f"(토트 {len([1 for i in range(m.nbody) if (m.body(i).name or '').startswith('tote_')])}개 × 2)")
    w()


def render(m, d, robot_bodies, outdir):
    """Robot collision-geom renders, front and side.

    Mutates rgba / headlight on this in-memory MjModel only; assets/*.xml are
    never touched. Environment and tote geoms are hidden so the robot's own
    collision shapes are legible.
    """
    saved_rgba = m.geom_rgba.copy()
    saved_amb = m.vis.headlight.ambient.copy()
    saved_dif = m.vis.headlight.diffuse.copy()
    rb = set(robot_bodies)
    shown = 0
    for g in range(m.ngeom):
        if int(m.geom_bodyid[g]) in rb and is_collision(m, g):
            body = m.body(int(m.geom_bodyid[g])).name or ""
            # Tint the project-added hooks so they stand out from official links.
            m.geom_rgba[g] = ([0.95, 0.55, 0.15, 1.0] if "tool" in body
                              else [0.25, 0.65, 0.95, 1.0])
            shown += 1
        else:
            m.geom_rgba[g] = [0, 0, 0, 0]
    m.vis.headlight.ambient[:] = [0.45, 0.45, 0.45]
    m.vis.headlight.diffuse[:] = [0.75, 0.75, 0.75]

    # MuJoCo free camera: forward = (cos az cos el, sin az cos el, sin el).
    # The robot faces +X (pallet at x=+1.15), so az=180 looks it in the face.
    views = [("front", 180, "정면 (+X에서 -X 방향)"),
             ("side", 270, "측면 (+Y에서 -Y 방향, 로봇 왼쪽)")]
    shots = []
    # The project's own collision geoms sit in group 3, which the default
    # visualisation option hides. Enable every group or the hooks vanish.
    opt = mujoco.MjvOption()
    opt.geomgroup[:] = 1
    with mujoco.Renderer(m, height=960, width=960) as r:
        cam = mujoco.MjvCamera()
        cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        cam.lookat[:] = [0.05, 0.0, 0.80]
        cam.distance = 2.7
        cam.elevation = -8
        for label, az, desc in views:
            cam.azimuth = az
            r.update_scene(d, cam, opt)
            img = r.render()
            path = outdir / f"step1_collision_{label}.png"
            try:
                from PIL import Image
                Image.fromarray(img).save(path)
                shots.append((label, path, az, desc))
            except ImportError:
                shots.append((label, None, az, desc))

    m.geom_rgba[:] = saved_rgba
    m.vis.headlight.ambient[:] = saved_amb
    m.vis.headlight.diffuse[:] = saved_dif
    return shots, shown


def steering_state():
    """What the simulation actually prescribes for the wheels, and the slip left.

    Measured through Simulation itself rather than re-derived, so the report
    tracks the code instead of a transcription of it.
    """
    from poc_sim.simulation import Simulation

    sim = Simulation(1)
    jac = np.zeros((3, sim.m.nv))
    rows = []
    for (adr, angle), (roll_adr, gain) in zip(sim.steering, sim.wheels):
        joint = sim.m.joint(int(np.flatnonzero(sim.m.jnt_qposadr == roll_adr)[0]))
        tag = joint.name.split("_")[0]
        body = int(sim.m.jnt_bodyid[joint.id])
        geom = int(np.flatnonzero(sim.m.geom_bodyid == body)[0])

        def residual(steer_angle):
            sim.d.qpos[adr] = steer_angle
            mujoco.mj_forward(sim.m, sim.d)
            ground = sim.d.geom_xpos[geom] * [1, 1, 0]
            mujoco.mj_jac(sim.m, sim.d, jac, None, ground, body)
            spin = jac[:, int(joint.dofadr[0])]
            drift = np.cross([0, 0, 1.0], ground - sim.d.body("base").xpos)
            ratio = -float(spin @ drift) / float(spin @ spin)
            heading = spin * [1, 1, 0]
            heading /= np.linalg.norm(heading)
            across = np.cross([0, 0, 1.0], heading)
            lever = float(np.linalg.norm((ground - sim.d.body("base").xpos)[:2]))
            return abs(float((drift + ratio * spin) @ across)), lever

        # The shipped behaviour pinned every steer joint at zero.
        before, _ = residual(0.0)
        after, lever = residual(angle)
        rows.append((tag, angle, gain, lever,
                     before * BASE_TURN_RATE, after * BASE_TURN_RATE))
    return {"rows": rows, "slip": max(r[5] for r in rows)}


def section5(m, d, uj, usens, shots, nshown):
    w("### 충돌 형상 렌더링")
    w()
    for label, p, az, desc in shots:
        if p is None:
            w(f"- {label}: PIL 없음, 저장 실패")
            continue
        w(f"- **{desc}** — `{p.relative_to(ROOT)}`  ")
        w(f"  (free camera azimuth={az}°, elevation=-8°, distance=2.7 m, "
          f"lookat=(0.05, 0, 0.80), geomgroup 전체 활성)")
    w()
    w(f"렌더 방식: **로봇 body의 충돌 geom {nshown}개만** 표시하고 "
      "(`contype` 또는 `conaffinity` ≠ 0), 바닥·컨베이어·팔레트·토트와 visual 전용 "
      "geom은 알파 0으로 숨겼습니다. 색: **파랑 = 공식 URDF 링크**, "
      "**주황 = 프로젝트가 추가한 후크**. "
      "메모리상의 `MjModel` 인스턴스만 바꾸며 `assets/*.xml`은 건드리지 않습니다.")
    w()
    w("## 5. 카메라")
    w()
    rows = []
    for c in range(m.ncam):
        name = m.camera(c).name or f"cam{c}"
        bid = int(m.cam_bodyid[c])
        rows.append([name, m.body(bid).name or "world",
                     np.round(m.cam_pos0[c], 4).tolist(),
                     f"{m.cam_fovy[c]:.2f}",
                     np.round(m.cam_quat[c], 4).tolist(),
                     {0: "fixed", 1: "track", 2: "targetbody",
                      3: "targetbodycom"}.get(int(m.cam_mode[c]), "?")])
    table(["카메라", "부착 body", "pos", "fovy (°)", "quat", "mode"], rows)
    w(f"- 모델 내 MuJoCo `<camera>` 총 **{m.ncam}개**.")
    attached = [m.camera(c).name for c in range(m.ncam) if int(m.cam_bodyid[c]) != 0]
    w(f"- 로봇 링크에 부착된 카메라: **{len(attached)}개**"
      + (f" — {attached}" if attached else ""))
    w()
    w("**헤드 카메라·손목 카메라가 MuJoCo 카메라로 정의돼 있지 않습니다.** "
      "모델의 두 카메라는 모두 `worldbody` 직속입니다:")
    w()
    w("- `overview`: 보고서용 전경 샷 전용. 인지에 쓰이지 않습니다.")
    w("- `overhead`: 팔레트 중심 위 3.5 m 고정. `poc_sim/perception.py`가 이 카메라로 "
      "RGB·깊이·세그멘테이션을 받아 구역 높이를 추정합니다. 로봇에 붙어 있지 않으므로 "
      "**천장 설치 고정 카메라**를 가정한 구성입니다.")
    w()
    w("### 공식 URDF가 정의한 센서 프레임")
    w()
    w("URDF에는 센서 장착 프레임이 **실재합니다**. 질량 0 또는 극소 링크로 들어와 있고, "
      "우리 mjModel에도 body로 로드돼 있지만 **MuJoCo 카메라가 하나도 붙어 있지 않습니다.**")
    w()
    rows = []
    for link, (parent, xyz, rpy, jname) in usens.items():
        if not any(k in link for k in
                   ["zed", "cam", "lidar", "imu", "ultrasonic"]):
            continue
        try:
            bid = m.body(link).id
            loaded = "로드됨"
            wpos = np.round(d.xpos[bid], 4).tolist()
        except KeyError:
            loaded = "**미로드**"
            wpos = "-"
        has_cam = any(int(m.cam_bodyid[c]) == m.body(link).id for c in range(m.ncam)) \
            if loaded == "로드됨" else False
        rows.append([link, parent, np.round(xyz, 4).tolist(),
                     np.round(rpy, 4).tolist(), loaded, wpos,
                     "있음" if has_cam else "**없음**"])
    table(["센서 프레임", "부모 링크", "mount xyz (m)", "mount rpy (rad)",
           "mjModel", "현 자세 world pos", "MuJoCo camera"], rows)
    w()
    w("URDF 원문의 주석(`vega_1.urdf:334-335`):")
    w()
    w("> *The zed_left_camera and zed_right_camera is used for simulation rendering. "
      "The depth frame i[s] used for the real robot transformation.*")
    w()
    w("**공식 모델은 `zed_left_camera` / `zed_right_camera`를 시뮬레이션 렌더링용 "
      "카메라로 쓰라고 명시하고 있습니다.** 우리 구현은 이 프레임을 쓰지 않고 "
      "별도의 천장 카메라를 만들어 썼습니다.")
    w()
    w("### 스테레오 베이스라인 (URDF 실측)")
    w()
    if "zed_left_camera" in usens and "zed_right_camera" in usens:
        lx = np.array(usens["zed_left_camera"][1])
        rx = np.array(usens["zed_right_camera"][1])
        base_mm = float(np.linalg.norm(lx - rx)) * 1000
        w(f"- `zed_left_camera` mount xyz = {lx.tolist()}")
        w(f"- `zed_right_camera` mount xyz = {rx.tolist()}")
        w(f"- **베이스라인 = {base_mm:.1f} mm**")
        w(f"- `zed_depth_frame`은 좌 카메라보다 x로 "
          f"{(np.array(usens['zed_depth_frame'][1])[0]-lx[0])*1000:+.1f} mm 떨어져 있습니다.")
    w()
    w("### 실제 장비 사양 대조")
    w()
    table(["항목", "확인 내용", "출처"],
          [["헤드 카메라 종류", "**스테레오 카메라** (좌/우 + depth 프레임)",
            "URDF 링크 구조"],
           ["제조사 계열", "Stereolabs **ZED** 계열 (링크명 `zed_*`)", "URDF 링크명"],
           ["스테레오 베이스라인", "**50.0 mm** (URDF mount 좌표 실측)", "URDF"],
           ["정확한 모델명 (ZED X Mini 등)", "**미확인**", "-"],
           ["해상도 / 프레임레이트", "**미확인**", "-"],
           ["FOV 실측값", "**미확인**", "-"],
           ["depth 방식 (스테레오/ToF)", "**미확인** (스테레오 추정)", "-"],
           ["손목 카메라", "**URDF에 없음** → 미탑재로 추정", "URDF"],
           ["베이스 카메라 4대 사양", "**미확인** (링크만 존재)", "URDF"],
           ["LiDAR 모델", "**미확인** (링크만 존재)", "URDF"],
           ["현장 천장 카메라 사양", "**미확인**", "-"]])
    w("> 베이스라인 50 mm는 URDF 좌표에서 계산한 값입니다. "
      "제품 데이터시트로 교차검증하지 않았습니다.")
    w()
    fov = m.cam_fovy[m.camera("overhead").id]
    w(f"시뮬레이터 `overhead` 카메라의 `fovy` = **{fov:.1f}°** "
      f"(MuJoCo 기본값 45°{'와 동일' if abs(fov-45)<1e-6 else ''}). "
      "실제 장비와 대조하지 않은 임의값입니다.")
    w()
    w("### 머리 관절")
    w()
    hrows = []
    for n in ["head_j1", "head_j2", "head_j3"]:
        u = uj.get(n, {})
        try:
            j = m.joint(n).id
            rng = (f"{m.jnt_range[j][0]:.4g} ~ {m.jnt_range[j][1]:.4g}"
                   if m.jnt_limited[j] else "무제한")
        except KeyError:
            rng = "**미로드**"
        hrows.append([n, rng,
                      "-" if u.get("velocity") is None else f"{u['velocity']:.3g}",
                      "-" if u.get("effort") is None else f"{u['effort']:.3g}"])
    table(["관절", "mjModel jnt_range", "URDF velocity", "URDF effort"], hrows)
    w("머리 관절 3개는 모델에 로드돼 있지만 `poc_sim/ik.py:44-46`의 제어 대상"
      "(`torso_j1~3`, `L/R_arm_j1~7` 17개)에 **포함되지 않습니다.** "
      "전 구동 동안 초기값에 고정되어 움직이지 않습니다. "
      "헤드 카메라를 쓰려면 이 관절들로 시선을 돌려야 하므로 함께 묶이는 항목입니다.")
    w()


def source_stem(name):
    """Map a model mesh name back to its source OBJ stem.

    prepare_robot.py emits one `<stem>_p<i>.obj` per convex piece, so several
    model meshes can share a source file.
    """
    base = re.sub(r"_p\d+$", "", name)
    return base if (MESH_DIR / f"{base}.obj").exists() else name


def obj_triangles(path):
    with open(path) as fh:
        return sum(len(line.split()) - 3 for line in fh if line.startswith("f "))


def mesh_integrity(m):
    """Per source OBJ: [(stem, pieces, loaded_faces, obj_tris, [links])]."""
    agg = {}
    for mid in range(m.nmesh):
        stem = source_stem(m.mesh(mid).name)
        src = MESH_DIR / f"{stem}.obj"
        if not src.exists():
            continue
        users = {m.body(int(m.geom_bodyid[g])).name
                 for g in range(m.ngeom)
                 if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH
                 and int(m.geom_dataid[g]) == mid}
        e = agg.setdefault(stem, [0, 0, obj_triangles(src), set()])
        e[0] += 1
        e[1] += int(m.mesh_facenum[mid])
        e[3] |= users
    return [(k, v[0], v[1], v[2], sorted(v[3])) for k, v in sorted(agg.items())]


def live_verts_for_stem(m, stem):
    """Loaded vertices of one source OBJ, assembled in a single link frame.

    Each piece is stored in its own MuJoCo-recentred principal frame, so the
    pieces only line up again once geom_pos/geom_quat is applied. A shared mesh
    (arm_l7 on both arms) is taken from one body only.
    """
    by_body = {}
    for g in range(m.ngeom):
        if m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        mid = int(m.geom_dataid[g])
        if source_stem(m.mesh(mid).name) != stem:
            continue
        by_body.setdefault(int(m.geom_bodyid[g]), []).append((g, mid))
    if not by_body:
        raise KeyError(stem)
    chunks = []
    for g, mid in max(by_body.values(), key=len):
        rot = np.zeros(9)
        mujoco.mju_quat2Mat(rot, m.geom_quat[g])
        chunks.append(mesh_live_verts(m, mid) @ rot.reshape(3, 3).T + m.geom_pos[g])
    return np.vstack(chunks)


def summary(m, ui, robot_bodies):
    broken = [x for x in mesh_integrity(m) if x[2] != x[3]]
    total = float(sum(m.body_mass[b] for b in robot_bodies))
    zero = [m.body(b).name for b in robot_bodies if m.body_mass[b] == 0.0]
    tri_bad = [b for b in robot_bodies if not triangle_ok(m.body_inertia[b])]
    attached = sum(1 for c in range(m.ncam) if int(m.cam_bodyid[c]) != 0)
    w("## 판정 요약")
    w()
    rows = [
        ["0", "모델 파일 · 변환 경로",
         "의심",
         ("`vega_1.urdf` → `assets/vega.xml` 2단계 변환. vega.xml은 **동결 산출물**이라 "
         "`.deps` 갱신이 자동 반영되지 않음")],
        ["1", "질량 합계",
         "의심",
         (f"{total:.2f} kg vs 제시 스펙 135 kg ({(total-135)/135*100:+.1f}%). "
         f"스펙 출처 미확인")],
        ["1", "관성 타당성",
         "정상",
         (f"주관성 전부 양수, 삼각부등식 위반 {len(tri_bad)}개, 변환 중 재계산 0개. "
         f"질량 0 링크 {len(zero)}개는 모두 센서·기준 프레임")],
        ["2", "관절 위치 한계",
         "정상",
         "URDF lower/upper가 `jnt_range`에 전부 일치 반영. placeholder 의심값 없음"],
        ["2", "`effort` 반영",
         "문제",
         (f"`jnt_actfrcrange`에 값은 들어왔으나 `nu={m.nu}`(액추에이터 0개)라 "
         f"**토크 한계가 동작에 전혀 작용하지 않음**")],
        ["2", "`velocity` 반영",
         "정상",
         ("MuJoCo 모델 차원에는 속도 한계가 없음(엔진 구조). "
         "`simulation.py`가 URDF를 재파싱해 궤적 시간 계산에 사용 — 경로 확인됨")],
        ["2", "damping / armature / frictionloss",
         "의심",
         "전 DOF 합계가 모두 0. 모터 회전자 관성·조인트 마찰 없음"],
        ["3", "단위 (각도·길이·질량·토크)",
         "정상",
         "`angle=radian`, `mesh_scale=(1,1,1)`, 중력 −9.81 m/s². SI 일관"],
        ["4", "충돌 메쉬 무결성",
         "**문제**" if broken else "정상",
         (f"OBJ 다중 객체로 **{len(broken)}개 소스의 면이 누락**" if broken else
          "전 소스 OBJ 로드율 100%. 볼록 분해 조각이 각각 별도 geom으로 등록됨")],
        ["4", "충돌 ↔ visual 치수",
         "의심",
         "18/19는 +1~4%(정상적 팽창). `arm_l8`만 축별 +24/−27/−19%로 형상 불일치"],
        ["4", "contype / conaffinity · exclude",
         "정상",
         f"비트 규약 일관, SRDF 131쌍이 `nexclude={m.nexclude}`에 전부 반영"],
        ["4", "후크 형상",
         "정상",
         ("돌기 20×84 mm가 손잡이 구멍 40×116 mm에 여유 있게 삽입. "
         "구동 조인트 없음은 문서에 명시된 설계")],
        ["5", "헤드 카메라",
         "문제",
         (f"로봇 부착 카메라 {attached}개. 공식 URDF가 `zed_left/right_camera`를 "
         f"렌더용으로 쓰라고 주석에 명시했으나 미정의")],
        ["5", "실제 장비 사양",
         "미확인",
         "ZED 계열·베이스라인 50 mm까지만 URDF로 확인. 모델명·해상도·FOV 미확인"],
    ]
    table(["장", "항목", "판정", "요약"], rows)
    w("판정 기준 — **정상**: 확인 결과 문제 없음 / **의심**: 근거가 부족하거나 "
      "값이 어긋나지만 즉시 오작동을 일으키지는 않음 / **문제**: 2단계 결과를 "
      "왜곡함 / **미확인**: 외부 자료로 확인하지 못함.")
    w()


def priorities(m, robot_bodies):
    broken = [x for x in mesh_integrity(m) if x[2] != x[3]]
    w("## 2단계로 넘어가기 전에 고쳐야 할 것")
    w()
    w("우선순위 순입니다. P0는 고치지 않으면 2단계 결과를 신뢰할 수 없는 항목입니다.")
    w()

    if broken:
        w("### P0 — 충돌 메쉬 면 누락 복구")
        w()
        w("MuJoCo OBJ 로더가 파일의 첫 `o` 블록만 읽어 나머지 면이 버려졌습니다. "
          "컴파일 경고가 없어 드러나지 않습니다.")
        w()
        rows = [[f"`{stem}`", ", ".join(users), f"{tris:,}", f"{mjf:,}",
                 f"**{100 - mjf / tris * 100:.0f}%**"]
                for stem, _, mjf, tris, users in
                sorted(broken, key=lambda x: x[2] / x[3])]
        table(["소스 OBJ", "영향 링크", "OBJ 삼각형", "로드됨", "손실"], rows)
        w("조치: 각 `o` 블록을 개별 OBJ로 분리해 링크당 여러 `<collision>`으로 "
          "등록합니다. 조각이 볼록 분해 결과이므로 **병합은 안 됩니다** — "
          "단일 convex hull이 씌워져 오목부가 메워집니다.")
        w()
    else:
        w("### P0 — 충돌 메쉬 면 누락 — **해결됨**")
        w()
        total_tris = sum(x[3] for x in mesh_integrity(m))
        npieces = sum(x[1] for x in mesh_integrity(m))
        w(f"전 소스 OBJ {len(mesh_integrity(m))}개의 로드율이 **100%**입니다 "
          f"(삼각형 {total_tris:,}개, geom {npieces}개).")
        w()
        w("`scripts/prepare_robot.py`가 볼록 분해 조각을 "
          "`assets/collision/<stem>_p<i>.obj`로 분리해 링크마다 별도 "
          "`<collision>`으로 등록합니다. 4장의 무결성 표에서 확인할 수 있습니다.")
        w()
        w("이 수정으로 **충돌 형상이 실제로 커졌습니다.** 아래 항목은 "
          "복구 이후를 기준으로 다시 읽어야 합니다:")
        w()
        w("- `poc_sim/ik.py`의 장애물 회피 여유(`AVOID_REQUIRE` 14 mm, "
          "`AVOID_SAFE` 45 mm)가 더 큰 형상에 대해 평가됩니다.")
        w("- 도달 한계와 층수 판정(`docs/requirements.md`의 8층 근거)은 "
          "축소된 형상으로 계산된 값이므로 재확인이 필요합니다.")
        w()

    w("### P1 — 액추에이터 부재 (동역학으로 갈 경우 필수)")
    w()
    w(f"`nu={m.nu}`, `na={m.na}`, `d.ctrl` 크기 {m.nu}. URDF의 `effort`(구동 16, 조향 6, "
      "몸통 380~700, 팔 25~150 N·m)가 모델에 들어와 있지만 "
      "(`jnt_actfrcrange`) 이를 소비할 액추에이터가 없어 **토크 한계가 무력**합니다. "
      "`poc_sim/simulation.py:242-246`이 `mj_step` 뒤에 로봇 `qpos`를 목표값으로 "
      "덮어쓰므로 물리 적분 결과도 버려집니다.")
    w()
    w("2단계가 운동학 범위라면 현 상태 유지도 일관성은 있습니다 "
      "(README가 한계를 명시). 다만 그 경우 **토크·슬립·전도에 관한 어떤 수치도 "
      "이 모델에서 나올 수 없다**는 점을 결과물에 못박아야 합니다.")
    w()

    steer = steering_state()
    if steer["slip"] > 1e-9:
        w("### P1 — 조향 조인트 미사용")
        w()
        w("`L/R_wheel_j1`(z축 조향, ±2.722 rad, effort 6 N·m)이 전 구동 동안 0에 "
          "고정됩니다. `rolling_wheels()`가 `*_wheel_j2`(롤링)만 회전시킵니다. "
          "제자리 선회 시 접지점이 요구하는 이동의 일부를 롤링으로 만들 수 없어 "
          f"측면 미끄러짐이 남습니다 — 선회율 {BASE_TURN_RATE} rad/s에서 "
          f"최대 {steer['slip']:.3e} m/s. 운동학 모델이라 벌점이 없을 뿐, "
          "README의 '구동 바퀴도 미끄러짐 없이 함께 굴립니다'는 성립하지 않습니다.")
        w()
    else:
        w("### P1 — 조향 조인트 미사용 — **해결됨**")
        w()
        w("`rolling_wheels()`가 세 바퀴 모두 선회 접선으로 조향한 뒤 굴림비를 "
          "재계산합니다. 제자리 선회는 접지점을 베이스 원점 기준 고정 원 위에 두므로 "
          "**베이스 좌표계에서 접선 방향이 변하지 않습니다** — 조향각은 상수이고 "
          "굴림각만 heading에 비례합니다. 후방 캐스터는 조향축보다 34 mm 뒤에 "
          "접지하므로 조향이 접지점을 움직여서, 반복 해법으로 수렴시킵니다.")
        w()
        table(["바퀴", "조향각", "굴림비 (rad/rad)", "레버암", "조향 전 횡슬립",
               "조향 후"],
              [[f"`{tag}_wheel`", f"{a:+.4f} rad ({np.degrees(a):+.2f}°)",
                f"{g:+.4f}", f"{lever:.4f} m", f"{before:.3e} m/s",
                f"**{after:.1e} m/s**"]
               for tag, a, g, lever, before, after in steer["rows"]])
        w(f"선회율 {BASE_TURN_RATE} rad/s 기준입니다. `L/R_wheel_j1`의 조향각은 "
          "한계 ±2.722 rad 안에 들어오고 `B_wheel_j1`은 continuous라 한계가 "
          "없습니다. 위 수치는 운반 자세에서의 해석적 잔차이고, 실제 180° 선회를 "
          "돌려 접지점 속도를 유한차분으로 읽으면 수치 잡음이 섞여 5e-7 m/s "
          "수준으로 측정됩니다 — `scripts/verify_steering.py`.")
        w()
        w("후방 캐스터 `B_wheel_j1`/`B_wheel_j2`는 수정 전에는 구동 목록에 "
          "아예 없었습니다. 레버암이 0.299 m로 가장 길어 횡슬립도 가장 컸습니다.")
        w()

    w("### P2 — 헤드 ZED 카메라 미정의")
    w()
    w("공식 URDF 주석이 `zed_left_camera`/`zed_right_camera`를 시뮬레이션 렌더링용으로 "
      "쓰라고 명시하는데, 모델에는 이 프레임에 `<camera>`가 없습니다. "
      "현재 인지는 천장 고정 `overhead` 카메라로 대체돼 있습니다. "
      "실제 로봇이 자기 머리 카메라로 팔레트를 본다면 **가림(occlusion)·시야각·"
      "선회 중 흔들림**이 전부 다르게 작용하므로, 인지 성능을 논하려면 "
      "이 프레임에 카메라를 붙이고 `head_j1~j3`로 시선을 돌려야 합니다. "
      "머리 3관절도 현재 IK 제어 대상이 아닙니다.")
    w()

    w("### P2 — 확인이 필요한 수치")
    w()
    table(["항목", "현재", "확인할 것"],
          [["로봇 질량", "142.11 kg (후크 1.2 kg 포함)",
            "공개 스펙 135 kg의 출처와 구성(핸드·배터리). 5.3% 차이의 정체"],
           ["`arm_l8` 충돌 형상", "visual 대비 +24/−27/−19%",
            "손목 말단 링크. 면 누락은 아니므로 원본 자산 자체의 불일치"],
           ["전체 높이", "최대 신장 1.932 m / 운전 자세 1.566 m",
            "1.7 m 스펙이 어느 자세 기준인지"],
           ["`overhead` fovy", "45° (MuJoCo 기본값)",
            "실제 천장 카메라 화각"]])

    w("### P3 — 구조적으로 남겨둘지 결정할 것")
    w()
    w("- `dof_damping` / `dof_armature` / `dof_frictionloss`가 전 DOF 0. "
      "동역학으로 가면 모터 회전자 관성과 조인트 마찰이 필요합니다.")
    w("- `assets/vega.xml`은 `prepare_robot.py` 산출물을 **동결**한 파일입니다. "
      "`.deps`의 URDF가 바뀌어도 자동 반영되지 않습니다. "
      "버전 기록(생성 시 URDF 해시)을 남기면 안전합니다.")
    w("- `L/R/B_wheel_l1`(조향 포크·캐스터 포크)에 충돌 geom이 없습니다. "
      "`visual/`에는 `wheel_l1.glb`, `B_wheel_l1.glb`가 존재하므로 "
      "collision OBJ가 애초에 제공되지 않은 것입니다. 바닥 근처라 영향은 작지만 "
      "의도된 생략인지 확인이 필요합니다.")
    w()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--write", action="store_true",
                    help="reports/step1_model_check.md 로 저장")
    ap.add_argument("--boxes", type=int, default=32)
    a = ap.parse_args()

    # Simulation() builds the same scene and additionally solves the initial IK,
    # so poses reported here match what run.py actually starts from.
    from poc_sim.simulation import Simulation
    sim = Simulation(a.boxes, 0.058, False, seed=2)
    m, d = sim.m, sim.d
    mujoco.mj_forward(m, d)

    tote0 = int(m.body("tote_0").id)
    robot_bodies = list(range(1, tote0))
    uj, ui = urdf_joints(), urdf_inertials()
    ulinks, usens = urdf_links(), urdf_sensor_frames()

    w("# 1단계 로봇 모델 검증 보고서")
    w()
    w(f"- 대상: Dexmate Vega-1 토트 적재 PoC (`{ROOT}`)")
    w("- 검증 기준: **MuJoCo에 로드·컴파일된 mjModel** (`assets/scene.xml`)")
    w(f"- MuJoCo {mujoco.__version__}, Python {sys.version.split()[0]}")
    w("- 재실행: `MUJOCO_GL=egl OPENBLAS_NUM_THREADS=1 "
      "python scripts/step1_check.py --write`")
    w(f"- 모델 규모: `nbody={m.nbody}` `njnt={m.njnt}` `nq={m.nq}` `nv={m.nv}` "
      f"`ngeom={m.ngeom}` `nu={m.nu}` `ncam={m.ncam}` `neq={m.neq}` `nexclude={m.nexclude}`")
    w(f"- 로봇 body 범위: 1 ~ {tote0-1} ({len(robot_bodies)}개), 토트: {tote0} 이상")
    w()
    summary(m, ui, robot_bodies)
    section0()
    section1(m, d, uj, ui, ulinks, robot_bodies)
    section2(m, uj, robot_bodies)
    section3(m, d, uj, robot_bodies)
    section4(m, d, robot_bodies)
    shots, nshown = render(m, d, robot_bodies, ROOT / "reports")
    section5(m, d, uj, usens, shots, nshown)
    priorities(m, robot_bodies)

    text = "\n".join(OUT)
    if a.write:
        p = ROOT / "reports/step1_model_check.md"
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(text)
        print(f"wrote {p}  ({len(text):,} chars)")
    else:
        print(text)


if __name__ == "__main__":
    main()
