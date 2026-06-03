"""Convert the TurtleBot4 URDF to USD using Isaac Sim's URDF Importer (headless).

Run with:  isaac-py ~/isaac_tb4/scripts/convert_urdf_to_usd.py
Output:    ~/isaac_tb4/usd/turtlebot4.usd
"""
import os

URDF = os.path.expanduser("~/isaac_tb4/usd/turtlebot4.urdf")
USD_OUT = os.path.expanduser("~/isaac_tb4/usd/turtlebot4.usd")

from isaacsim import SimulationApp
sim_app = SimulationApp({"headless": True})

from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.asset.importer.urdf")
sim_app.update()

import omni.kit.commands

# Build an import config tuned for a mobile robot
status, cfg = omni.kit.commands.execute("URDFCreateImportConfig")
cfg.merge_fixed_joints = True      # collapse welded links -> fewer prims
cfg.convex_decomp = False          # keep simple collision
cfg.fix_base = False               # mobile base must be free to move
cfg.make_default_prim = True
cfg.self_collision = False
cfg.distance_scale = 1.0           # URDF is already in meters
cfg.density = 0.0                  # use inertial from URDF
cfg.create_physics_scene = True

print(f"[convert] importing {URDF}")
res = omni.kit.commands.execute(
    "URDFParseAndImportFile",
    urdf_path=URDF,
    import_config=cfg,
    dest_path=USD_OUT,
)
print(f"[convert] command result: {res}")
print(f"[convert] wrote {USD_OUT}  exists={os.path.exists(USD_OUT)}")

sim_app.close()
