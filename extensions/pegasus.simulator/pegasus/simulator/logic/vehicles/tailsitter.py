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
        """

        # Stage prefix of the vehicle when spwaning in the world
        self.stage_prefix = "tailsitter"
        # self.usd_file = ROBOTS["Tailsitter"]

        # Rotor model
        self.num_rotors = 2
        self.rotor_positions = [...]
        self.rotor_axes = [...]
        self.rotor_directions = [...]
        self.max_rotor_velocity = ...
        self.thrust_curve = ...
        self.torque_coefficient = ...

        # Basic aero
        self.wing_area = ...
        self.wing_span = ...
        self.chord = ...
        self.CL_0 = ...
        self.CL_alpha = ...
        self.CL_max = ...
        self.CD_0 = ...
        self.Cm_alpha = ...

        # Control surfaces
        self.Cm_elevator = ...
        self.Cl_aileron = ...
        self.Cn_rudder = ...

        # Transition
        self.transition_airspeed = ...
        self.hover_blend_airspeed = ...
        self.forward_blend_airspeed = ...

        # Sensors / backends
        self.sensors = [...]
        self.backends = [...]

class Tailsitter(Vehicle):
    """
    Tailsitter class - Defines a base interface for creating VTOL vehicle with dual fixed motors and two elevons.
    """

    def __init__(self, config: TailsitterConfig, backend_config: ArduPilotMavlinkBackendConfig):
        # Initialize the base Vehicle class
        super().__init__(config, backend_config)
        # TODO: do i want inheritance from parent?

    def update(self, dt):
        # Call the base class update to handle backend communication and sensor updates
        super().update(dt)

        # commands = self.get_commands_from_backend()
        # regime = self.compute_flight_regime()
        # rotor_ft = self.compute_rotor_forces_and_moments(commands, dt)
        # aero_ft = self.compute_aero_forces_and_moments(commands, dt)
        # total_ft = self.blend_forces_and_moments(rotor_ft, aero_ft, regime)
        # self.apply_total_forces_and_moments(total_ft)
        # self.update_actuator_visuals(commands)
