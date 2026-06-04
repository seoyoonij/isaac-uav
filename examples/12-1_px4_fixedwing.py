#!/usr/bin/env python
"""
| File: 12-1_px4_fixedwing.py
| Author: Seoyoon Jung
| Description: Patched 12_px4_fixedwing.py to use the PX4 backend instead of ArduPilot.
|              Adapted from 1_px4_single_vehicle.py and 12_ardupilot_fixedwing.py.
"""

"""
Launch order
1. WSL2: start PX4 
    ``cd ~/isaac_sim_project/PX4-Autopilot
      make px4_sitl_default none_iris``
2. Open QGC, wait for auto-connect
3. Isaac Sim: run this script
    ``cd C:\isaac-uav
      C:\isaac-uav\app\python.bat .\examples\12-1_px4_fixedwing.py``

"""
# Imports to start Isaac Sim from this script
import carb
from isaacsim import SimulationApp

# Start Isaac Sim's simulation environment
# Note: this simulation app must be instantiated right after the SimulationApp import, otherwise the simulator will crash
# as this is the object that will load all the extensions and load the actual simulator.
simulation_app = SimulationApp({"headless": False})

import omni.timeline
from omni.isaac.core.world import World

# Import local extension sources before any installed pegasus package
from pathlib import Path
import sys
repo_root = Path(__file__).resolve().parents[1]
uav_extensions = repo_root / "extensions"
uav_simulator = uav_extensions / "pegasus.simulator"
for p in (uav_extensions, uav_simulator):
    p_str = str(p)
    if p_str not in sys.path:
        sys.path.insert(0, p_str)

# Import the Pegasus API for simulating FixedWing
from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackend, PX4MavlinkBackendConfig
from pegasus.simulator.logic.vehicles.fixedwing import FixedWing, FixedWingConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
# Auxiliary scipy and numpy modules
import os.path
from scipy.spatial.transform import Rotation

class FixedWingPX4App:
    """
    Isaac Sim standalone App for FixedWing using PX4 backend.
    """

    def __init__(self):

        # Acquire the timeline that will be used to start/stop the simulation
        self.timeline = omni.timeline.get_timeline_interface()

        # Start the Pegasus Interface
        self.pg = PegasusInterface()

        # Acquire the World, .i.e, the singleton that controls that is a one stop shop for setting up physics, 
        # spawning asset primitives, etc.
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        # Launch one of the worlds provided by NVIDIA
        self.pg.load_environment(SIMULATION_ENVIRONMENTS["Default Environment"])

        # Create the fixed-wing aircraft
        self.create_fixedwing_vehicle()

        # Reset the simulation environment so that all articulations are initialized
        self.world.reset()

        # Auxiliary variable for the timeline callback example
        self.stop_sim = False

        print("Fixed-wing PX4simulation initialized.")


    def create_fixedwing_vehicle(self):
        """
        Create a single fixed-wing aircraft with configured backend
        """
        config = FixedWingConfig()
        
        # Propeller/Motor settings
        config.prop_max_thrust = 100.0          # Maximum thrust in Newtons (Targeting ~1kg mass)
        config.prop_max_rpm = 10000.0          # Maximum RPM
        config.prop_thrust_coefficient = 0.000075  # Thrust coefficient
        config.prop_rotation_dir = 1           # 1: CCW, -1: CW
        
        # Aircraft geometry
        config.wing_area = 2.36                # Wing area (m²) (Span 4.46 * Chord 0.53)
        config.wing_span = 4.46                 # Wing span (m)
        config.chord = 0.53                    # Mean aerodynamic chord (m)
        
        # Aerodynamic coefficients (adjust based on your aircraft)
        config.CL_0 = 0.3                      # Zero AoA lift coefficient
        config.CL_alpha = 4.0                  # Lift curve slope
        config.CL_max = 1.5                    # Stall limit (upper)
        config.CD_0 = 0.025                    # Parasitic drag
        
        # Control surface effectiveness
        config.Cm_elevator = -1.5              # Elevator pitch moment
        config.Cl_aileron = 0.3                # Aileron roll moment
        config.Cn_rudder = -0.05               # Rudder yaw moment

        # Simulation
        # Manual      : User provides forces. Aerodynamics are NOT calculated. Useful for frame debugging.
        # Thrust Only : User provides forces. Aerodynamics are calculated. Useful for Aerodynamics Coefficient debugging
        # Autonomous  : Needs backend, no user control needed. Aerodynamics are calculated. Production mode.
        config.simulation_mode = 'autonomous' # autonomous, thrust_only, manual
        #config.debug_mode = True
        
        px4_config = PX4MavlinkBackendConfig({
            "vehicle_id": 0,
            "px4_autolaunch": False, # Launch PX4 manually in WSL2
            "px4_dir": self.pg.px4_path,
            "px4_vehicle_model": "none_plane" # Use custom PX4 airframe 'none_plane' for fixed-wing
        })
        
        # Combine backends
        config.backends = [
            PX4MavlinkBackend(config=px4_config),  # Uncomment for Ardupilot
        ]

        self.aircraft = FixedWing(
            stage_prefix="/World/fixedwing0",
            usd_file=ROBOTS['Fixed Wing'],
            vehicle_id=0,
            init_pos=[0.0, 0.0, 1.0],
            init_orientation=Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
            config=config
        )
        
        print("Fixed-wing aircraft created.")

    def run(self):
        """
        Method that implements the application main loop, where the physics steps are executed.
        """

        # Start the simulation
        self.timeline.play()

        # The "infinite" loop
        while simulation_app.is_running() and not self.stop_sim:

            # Update the UI of the app and perform the physics step
            self.world.step(render=True)
        
        # Cleanup and stop
        carb.log_warn("FixedWing PX4 App is closing.")
        self.timeline.stop()
        simulation_app.close()

def main():

    # Instantiate the template app
    app = FixedWingPX4App()

    # Run the application loop
    app.run()

if __name__ == "__main__":
    main()
