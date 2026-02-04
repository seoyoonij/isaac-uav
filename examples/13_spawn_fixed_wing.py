from isaacsim import SimulationApp

# 1. Start the Application
simulation_app = SimulationApp({"headless": False})

import omni.usd
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.nucleus import get_assets_root_path
from pxr import Sdf
import os

# Enter your specific UAV folder path here
base_folder = "../extensions/pegasus.simulator/pegasus/simulator/assets/Robots/yoda_fixed_wing"

# UAV Files
main_usd_path = f"{base_folder}/yoda_fixed_wing_flattened.usd"
dependency_path = f"{base_folder}/yoda_fixed_wing.usdc"

# LOCATE ENVIRONMENT FROM ISAAC SIM SERVER
print("Searching for Isaac Sim asset server (Nucleus)...")
assets_root_path = get_assets_root_path()

if assets_root_path is None:
    print("ERROR: Could not access Isaac Sim assets! Check your internet connection.")
    env_usd_path = None
else:
    # Path to Grid Room on the official server
    env_usd_path = assets_root_path + "/Isaac/Environments/Grid/gridroom_curved.usd"
    print(f"Environment found: {env_usd_path}")

# We are fixing the incorrect hardcoded path inside the UAV file in memory
# print("Repairing UAV file in memory...")
# layer = Sdf.Layer.FindOrOpen(main_usd_path)
# if layer:
#     for prim_spec in layer.rootPrims:
#         if prim_spec.hasReferences:
#             # Clear old references and add the correct local path
#             prim_spec.referenceList.ClearReferences()
#             prim_spec.referenceList.AddReference(dependency_path)
#             break

# SETUP THE SCENE
world = World()

# Add Environment 
if env_usd_path:
    add_reference_to_stage(usd_path=env_usd_path, prim_path="/World/Env")
else:
    # Add default ground plane if server is unavailable
    world.scene.add_default_ground_plane()

# Add UAV 

uav_prim_path = "/World/MyUAV"
add_reference_to_stage(usd_path=main_usd_path, prim_path=uav_prim_path)

world.reset()
print(f"Simulation is ready!")

# RUN LOOP
while simulation_app.is_running():
    world.step(render=True)

simulation_app.close()