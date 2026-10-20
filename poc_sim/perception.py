"""Overhead RGB-D + simulator semantic camera; no reads of object poses/counts."""

import mujoco
import numpy as np

from .layout import PALLET_TOP, SLOTS, TOTE_HEIGHT
from .props import RIM_TOP

# Height of a tote's upper rim above its body centre, as the depth camera sees it.
RIM_OFFSET = RIM_TOP


class OverheadCamera:
    def __init__(self, model, pitch, size=256):
        self.model = model
        self.pitch = pitch
        self.size = size
        self.renderer = mujoco.Renderer(model, height=size, width=size)
        self.camera = model.camera("overhead").id
        # Simulator-provided semantic mask, not a learned vision detector.
        self.tote_geoms = np.array(
            [
                g
                for g in range(model.ngeom)
                if model.geom(g).name.startswith("tote_visual_")
            ]
        )
        self.latest = None

    def observe(self, data):
        r = self.renderer
        r.disable_depth_rendering()
        r.disable_segmentation_rendering()
        r.update_scene(data, camera="overhead")
        rgb = r.render().copy()
        r.enable_depth_rendering()
        depth = r.render().copy()
        r.disable_depth_rendering()
        r.enable_segmentation_rendering()
        seg = r.render().copy()
        r.disable_segmentation_rendering()
        is_tote = np.isin(seg[:, :, 0], self.tote_geoms) & (
            seg[:, :, 1] == mujoco.mjtObj.mjOBJ_GEOM
        )
        # The calibrated overhead camera is axis-aligned and looks along world -Z.
        f = 0.5 * self.size / np.tan(np.deg2rad(self.model.cam_fovy[self.camera]) / 2)
        v, u = np.indices(depth.shape)
        pos = data.cam_xpos[self.camera]
        x = pos[0] + (u + 0.5 - self.size / 2) * depth / f
        y = pos[1] - (v + 0.5 - self.size / 2) * depth / f
        z = pos[2] - depth
        counts = []
        tops = []
        visibility = []
        for sx, sy in SLOTS:
            roi = (np.abs(x - sx) < 0.215) & (np.abs(y - sy) < 0.265)
            pixels = z[roi & is_tote]
            if len(pixels) > 20:
                # Upper rim, rather than the visible interior bottom, defines stack height.
                top = float(np.percentile(pixels, 99))
                count = max(
                    0,
                    1
                    + round(
                        (top - PALLET_TOP - TOTE_HEIGHT / 2 - RIM_OFFSET) / self.pitch
                    ),
                )
            else:
                top = PALLET_TOP
                count = 0
            # Empty slots must have visible pallet surface; an occluded slot is unknown.
            pallet_pixels = np.count_nonzero(roi & (np.abs(z - PALLET_TOP) < 0.015))
            valid = len(pixels) > 20 or pallet_pixels > 15
            counts.append(count if valid else -1)
            tops.append(top)
            visibility.append(valid)
        self.latest = {
            "time": float(data.time),
            "counts": counts,
            "top_z": tops,
            "visible": visibility,
            "source": "MuJoCo RGB-D + semantic segmentation",
        }
        self.rgb = rgb
        self.depth = depth
        self.segmentation = seg
        return self.latest

    def close(self):
        self.renderer.close()
