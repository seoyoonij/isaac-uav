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
from turtle import mode

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

        # PROP/ROTOR CONFIG (for each. Prop-thrust,cruise; Rotor-lift,hover)
        self.num_rotors = 2
        self.rotor_positions = [...] # Distance from CG
        self.rotor_axes = [[1,0,0], [1,0,0]] # Body X-axis
        self.rotor_directions = [1, -1] # Counter-rotate: top-inward (1: CCW, -1: CW from behind)
        self.rotor_inertia = ... # Gyroscopic precssion
        self.prop_joint_name = ["left_prop_joint", "right_prop_joint"]
        self.prop_visual_max_dof_speed = 60.0

        self.prop_max_rpm = 8000.0
        self.prop_thrust_coefficient = 0.00001 # Thrust = Kt * RPM^2
        self.prop_torque_coefficient = 0.0000002 # Torque = Kq * RPM^2

        self.prop_disk_area = ... # For prop wash effects
        self.elevon_submerged_fraction = ... # How much control authority

        # AIRCRAFT GEOMETRY
        self.wing_area = ...
        self.wing_span = ...
        self.chord = ...
        self.air_density = 1.225 # kg/m^3 at sea level
        self.aero_center = ... # Aerodynamic center for force application in body frame (m)

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
        self.transition_airspeed = ... # Midpoint speed
        self.hover_blend_airspeed = ... # Speed when beginning transition
        self.cruise_blend_airspeed = ... # Speed when ending transition
        self.transition_duration = ...
        self.transition_pitch_target = ... # Target pitch angle in transition (degrees)\
        self.transition_blend_type = 'linear' # linear, sigmoid, etc.
        self.transition_pitch_rate = ... # Pitch down slowly
        self.transition_thrust_scale = ... # Extra throttle boost to maintain altitude
        self.abort_airspeed_threshold = ...

        # SENSORS
        self.sensors = [Barometer(), IMU(), Magnetometer(), GPS()]
        self.graphical_sensors = []
        self.graphs = []

        # CONTROL BACKENDS
        self.backends = []

        # SIMULATION MODE
        # Modes:
        # - autonomous: full simulation with backend control and physics
        # - thrust_only: manual motor command, simulated aerodynamics and forces
        # - manual: bypass aerodynamics, directly apply raw forces and torques
        self.simulation_mode = "autonomous"
        self.debug_mode = False

        # Optional fallback mass for diagnostics when USD mass cannot be read.
        self.mass_kg = 1.0

        # Rate of takeoff viability diagnostics [Hz]
        self.viability_log_rate_hz = 10.0

        # Control input sign mapping (useful when backend and sim conventions differ).
        # Defaults assume backend commands are FRD-like while the simulated body is FLU:
        # roll about X unchanged, pitch and yaw signs flipped.
        self.aileron_sign = 1.0
        self.elevator_sign = -1.0
        self.throttle_sign = 1.0
        self.rudder_sign = -1.0
        # TODO: change if tailless


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

        mode = self._simulation_mode

        if mode == "manual":
            # 1. Get raw forces/torques from UI
            # 2. Apply directly to rigid body, skip aerodynamics
            pass
        elif mode == "thrust_only":
            # 1. Get motor commands from backend
            # 2. Compute passive aero forces (lift, drag, moments)
            # 3. Combine UI thrust + passive aero
            pass
        elif mode == "autonomous":
            # 1. Get multi-channel inputs from backend
            # 2. Run the tailsitter mixer to motor RPMs and elevon angles
            # 3. Compute acitve aero + prop-wash + motor torques
            pass
