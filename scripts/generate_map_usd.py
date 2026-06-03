"""Build a ground-floor USD (floor + collidable walls) from a ROS occupancy map.

Reads a map_server .yaml/.pgm, converts occupied cells into box walls placed in
the SAME world frame as the map (so it lines up with Nav2 / the map origin),
adds a ground plane, and writes a USD with physics colliders.

Usage (env vars for standalone pxr are set by run_generate_map.sh):
    python generate_map_usd.py <map.yaml> <out.usd> [wall_height]
"""
import sys, os, yaml
import numpy as np
from PIL import Image
from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf, UsdShade

MAP_YAML = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser("~/isaac_tb4/maps/A-1_map.yaml")
OUT_USD  = sys.argv[2] if len(sys.argv) > 2 else os.path.expanduser("~/isaac_tb4/scenes/A-1_ground.usd")
WALL_HEIGHT = float(sys.argv[3]) if len(sys.argv) > 3 else 0.6   # metres, above the lidar
FLOOR_THICK = 0.05
WALL_MARGIN = 0.0   # extra metres around the map for the floor

# ---- read map -------------------------------------------------------------
with open(MAP_YAML) as f:
    meta = yaml.safe_load(f)
res = float(meta["resolution"])
ox, oy, _ = meta["origin"]
occ_thr = float(meta.get("occupied_thresh", 0.65))
negate = int(meta.get("negate", 0))
img_path = os.path.join(os.path.dirname(os.path.abspath(MAP_YAML)), meta["image"])

img = np.array(Image.open(img_path).convert("L")).astype(np.float32)
H, W = img.shape
# ROS occupancy: occ = (255 - p)/255 if negate==0 else p/255
occ_val = (img / 255.0) if negate else (255.0 - img) / 255.0
occupied = occ_val > occ_thr   # boolean HxW, row 0 = top of image
print(f"[map] {W}x{H} cells, res={res} m, origin=({ox},{oy}), occupied={int(occupied.sum())}")

# ---- greedy rectangle merge of occupied cells -----------------------------
used = np.zeros_like(occupied, dtype=bool)
rects = []  # (i0, i1, j0, j1) inclusive, image coords
for j in range(H):
    for i in range(W):
        if not occupied[j, i] or used[j, i]:
            continue
        # expand width along the row
        i1 = i
        while i1 + 1 < W and occupied[j, i1 + 1] and not used[j, i1 + 1]:
            i1 += 1
        # expand height while the whole [i..i1] span stays occupied & free
        j1 = j
        while j1 + 1 < H and occupied[j1 + 1, i:i1 + 1].all() and not used[j1 + 1, i:i1 + 1].any():
            j1 += 1
        used[j:j1 + 1, i:i1 + 1] = True
        rects.append((i, i1, j, j1))
print(f"[map] merged into {len(rects)} wall boxes")

# ---- build stage ----------------------------------------------------------
stage = Usd.Stage.CreateNew(OUT_USD)
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
world = UsdGeom.Xform.Define(stage, "/World")
stage.SetDefaultPrim(world.GetPrim())

def make_box(path, cx, cy, cz, sx, sy, sz, color):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.GetSizeAttr().Set(1.0)                       # unit cube, extent -0.5..0.5
    cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    x = UsdGeom.Xformable(cube)
    x.AddTranslateOp().Set(Gf.Vec3d(cx, cy, cz))
    x.AddScaleOp().Set(Gf.Vec3f(sx, sy, sz))
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())     # static collider (no rigid body)
    return cube

# ground plane (floor): top surface at z=0
fw, fh = W * res + 2 * WALL_MARGIN, H * res + 2 * WALL_MARGIN
fcx, fcy = ox + W * res / 2.0, oy + H * res / 2.0
make_box("/World/GroundPlane", fcx, fcy, -FLOOR_THICK / 2.0,
         fw, fh, FLOOR_THICK, (0.35, 0.35, 0.38))

# walls
UsdGeom.Scope.Define(stage, "/World/Walls")
for n, (i0, i1, j0, j1) in enumerate(rects):
    sx = (i1 - i0 + 1) * res
    sy = (j1 - j0 + 1) * res
    cx = ox + (i0 + i1 + 1) / 2.0 * res
    cy = oy + (2 * H - j0 - j1 - 1) / 2.0 * res
    make_box(f"/World/Walls/wall_{n:03d}", cx, cy, WALL_HEIGHT / 2.0,
             sx, sy, WALL_HEIGHT, (0.6, 0.6, 0.62))

stage.GetRootLayer().Save()
print(f"[map] wrote {OUT_USD}  (floor {fw:.2f}x{fh:.2f} m, {len(rects)} walls, h={WALL_HEIGHT} m)")
