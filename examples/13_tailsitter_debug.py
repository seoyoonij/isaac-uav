"""
Purpose: Verify tailsitter implementation in isolation
     - Load environment, Spawn tailsitter, Step simulation

n.b. No backend

VERIFICATION CHECKLIST
1. Import and load python extension
2. Vehicle spawn in expected pose
3. Physics body (e.g. gravity)
4. Force application: Constant body force, sign
5. Update callback every frame
"""

# Imports to start Isaac Sim from this script
import carb
from isaacsim import SimulationApp
# Start Isaac Sim's simulation environment
# Note: this simulation app must be instantiated right after the SimulationApp import
simulation_app = SimulationApp({"headless": False})

import omni.timeline
from omni.isaac.core.world import World

# Import local extension sources before any installed pegasus package
from pathlib import Path
import sys
repo_root = Path(__file__).resolve().parents[1]
utils_dir = Path(__file__).resolve().parent / "utils"
uav_extensions = repo_root / "extensions"
uav_simulator = uav_extensions / "pegasus.simulator"

for p in (utils_dir, uav_extensions, uav_simulator):
    p_str = str(p)
    if p_str not in sys.path:
        sys.path.insert(0, p_str)

# Import the Pegasus API
from pegasus.simulator.params import SIMULATION_ENVIRONMENTS, ROBOTS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

# Auxiliary scipy and numpy modules
from scipy.spatial.transform import Rotation

# Import the Tailsitter class
from pegasus.simulator.logic.vehicles.tailsitter import Tailsitter, TailsitterConfig


class TailsitterDebugApp:
    def __init__(self):
        self.timeline = omni.timeline.get_timeline_interface()

        self.pg = PegasusInterface()
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        self.pg.load_environment(SIMULATION_ENVIRONMENTS["Default Environment"])

        self.expected_pos = [0.0, 0.0, 1.0]
        self.expected_orientation = Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat()

        self.create_tailsitter_vehicle()

        self.world.reset()
        self.step_count = 0
        print("Tailsitter debug app initialized.")

    def create_tailsitter_vehicle(self):
        config = TailsitterConfig()
        config.simulation_mode = "manual"

        placeholder_usd = ROBOTS.get("Fixed Wing")
        if placeholder_usd is None:
            raise KeyError('ROBOTS["Fixed Wing"] was not found in the simulator asset map.')

        self.aircraft = Tailsitter(
            stage_prefix="/World/tailsitter0",
            usd_file=placeholder_usd,
            vehicle_id=0,
            init_pos=self.expected_pos,
            init_orientation=self.expected_orientation,
            config=config,
        )

        print(f"Spawn requested at pos={self.expected_pos}, orientation={self.expected_orientation.tolist()}")
        print(f"Using placeholder USD: {placeholder_usd}")

    def run(self):
        self.timeline.play()
        print("Simulation started.")

        while simulation_app.is_running():
            self.world.step(render=True)
            self.step_count += 1

            if self.step_count == 1:
                print(f"First step state position: {self.aircraft.state.position}")

        carb.log_warn("Tailsitter debug app is closing.")
        self.timeline.stop()
        simulation_app.close()


def main():
    app = TailsitterDebugApp()
    app.run()


if __name__ == "__main__":
    main()
