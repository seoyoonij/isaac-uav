"""
| File: tailsitter.py
| Author: Seoyoon Jung
| License: BSD-3-Clause
| Description: Definition of the Tailsitter class with dual fixed motors and two elevons.
               Hybrid of multirotor + fixed-wing:
               - propulsion/rotor command flow: multirotor-based
               - aerodynamic forces and control surfaces: fixed-wing-based
               - additional hover-to-cruise transition logic
"""
import numpy as np
from pxr import Usd, UsdPhysics, UsdGeom, Gf
from scipy.spatial.transform import Rotation

from omni.isaac.dynamic_control import _dynamic_control

# The vehicle interface
from pegasus.simulator.logic.vehicles.vehicle import Vehicle

# Mavlink interface (veya kendi backend'iniz)
from pegasus.simulator.logic.backends.ardupilot_mavlink_backend import ArduPilotMavlinkBackend, ArduPilotMavlinkBackendConfig

# Sensors
from pegasus.simulator.logic.sensors import Barometer, IMU, Magnetometer, GPS

# Dynamics
from pegasus.simulator.logic.dynamics import LinearDrag

# Force UI for coordinate debugging
from pegasus.simulator.ui.force_debugger import ForceControlWindow, DebugVisualizer

# Log
import carb
import csv
import os
import time

# Constants for physical limits
MIN_THROTTLE = -500.0
MAX_THROTTLE = 500.0

MIN_FORCE = -1000.0
MAX_FORCE = 1000.0

MIN_MOMENTS = -500.0
MAX_MOMENTS = 500.0

class TailsitterConfig:
    """
    A data class that is used for configuring a Tailsitter Aircraft
    """
    def __init__(self):
        """
        Initialization of the TailsitterConfig class
        Adapted from fixedwing.py
        """

        # Stage prefix of the vehicle when spwaning in the world
        self.stage_prefix = "tailsitter"
        # self.usd_file = ROBOTS["Tailsitter"]

        # ROTOR CONFIG
        self.num_rotors = 2
        self.rotor_positions = [...]
        self.rotor_axes = [...]
        self.rotor_directions = [...]
        self.max_rotor_velocity = ...
        self.thrust_curve = ...
        self.torque_coefficient = ...

        # AIRCRAFT GEOMETRY
        self.wing_area = ...
        self.wing_span = ...
        self.chord = ...
        self.air_density = 1.225 # kg/m^3 at sea level
        self.aero_center = ... # aerodynamic center for force application in body frame (m)

        # AERODYNAMIC FORCE MODEL
        self.CL_0 = ... # Zero angle-of-attack lift coefficient
        self.CL_alpha = ... # Lift curve slope (per radian)
        self.CL_max = ... # Maximum lift coefficient (stall limit)

        self.CD_0 = ... # Zero-lift drag coefficient (parasitic drag)
        self.CD_alpha = ... # Drag due to angle-of-attack (induced drag)
        self.CD_alpha2 = ... # Quadratic drag for high angles of attack

        self.Cy_beta = ... # Side force due to sideslip

        self.Cm_0 = ... # Zero angle-of-attack pitching moment coefficient
        self.Cm_alpha = ... # Pitching moment due to angle-of-attack

        self.Cl_beta = ... # Roll moment due to sideslip
        self.Cl_p = ... # Roll damping due to roll rate
        self.Cl_r = ... # Roll-yaw coupling due to yaw rate

        self.Cn_beta = ... # Yaw moment due to sideslip
        self.Cn_p = ... # Yaw-roll coupling due to roll rate    
        self.Cn_r = ... # Yaw damping due to yaw rate

        self.Cm_q = ... # Pitch damping due to pitch rate

        # CONTROL SURFACE MOMENTS
        self.Cl_aileron = ... # Roll moment
        self.Cm_elevator = ... # Pitch moment
        self.Cn_rudder = ... # Yaw moment

        self.CL_elevator = ... # Lift change due to elevator (pitch in hover)
        self.CY_rudder = ... # Side force due to rudder (sideslip in hover)
        
        # # if tailless, below instead:
        # self.Cm_elevon = ... # Total pitch moment
        # self.Cl_elevon_diff = ... # Roll in hover, Yaw in cruise
        # self.Cl_elevon_sym = ... # Pitch in hover
        # self.Cn_elevon_diff = ... # Rudder against adverse yaw: slip due to higher lift, higher drag

        # DRAG CONFIG
        self.drag = LinearDrag([0.1, 0.1, 0.1]) # linear drag for body frame drag effects

        # TRANSITION CONFIG
        self.transition_airspeed = ...
        self.hover_blend_airspeed = ...
        self.cruise_blend_airspeed = ...
        self.transition_duration = ...
        # TODO whatelse

        # SENSORS
        self.sensors = [Barometer(), IMU(), Magnetometer(), GPS()]
        self.graphical_sensors = []
        self.graphs = []

        # CONTROL BACKENDS
        self.backends = []

        # SIMULATION MODE
        # Modes:
        # - hover
        # - transition_blend
        # - cruise
        # - manual
        self.simulation_mode = 'manual'
        self.debug_mode = False

class Tailsitter(Vehicle):
    """
    Tailsitter class - Defines a base interface for creating VTOL vehicle with dual fixed motors and two elevons.
    """

    def __init__(self, config: TailsitterConfig, backend_config: ArduPilotMavlinkBackendConfig):
        # Initialize the base Vehicle class
        super().__init__(config, backend_config)
        # TODO: do i want inheritance from parent?

    def update(self, dt):
        # Adapted from multirotor.py
        # Handle backend communication and sensor updates for physics simulation
        super().update(dt)

        # commands = self.get_commands_from_backend()
        # regime = self.compute_flight_regime()
        # rotor_ft = self.compute_rotor_forces_and_moments(commands, dt)
        # aero_ft = self.compute_aero_forces_and_moments(commands, dt)
        # total_ft = self.blend_forces_and_moments(rotor_ft, aero_ft, regime)
        # self.apply_total_forces_and_moments(total_ft)
        # self.update_actuator_visuals(commands)
