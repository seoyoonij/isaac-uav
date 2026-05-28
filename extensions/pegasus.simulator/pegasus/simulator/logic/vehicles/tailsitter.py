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
        # USD file path of the vehicle
        self.usd_file = ""
        
        # ===ADD PARAMETERS===

        # SENSORS
        self.sensors = [Barometer(), IMU(), Magnetometer(), GPS()]
        self.graphical_sensors = []
        self.graphs = []

        # CONTROL BACKENDS
        self.backends = []

        # DRAG CONFIG
        self.drag = LinearDrag([0.1, 0.1, 0.1])

        # SIMULATION MODE
        # Modes:
        # - autonomous: full simulation with backend control and physics
        # - thrust_only: manual motor command, simulated aerodynamics and forces
        # - manual: bypass aerodynamics, directly apply raw forces and torques
        self.simulation_mode = "manual"
        self.debug_mode = False


class Tailsitter(Vehicle):
    """
    Tailsitter class - Defines a base interface for creating VTOL vehicle with dual fixed motors and two elevons.
    """
    def __init__(
        self,
        # Simulation specific configurations
        stage_prefix: str = "tailsitter",
        usd_file: str = "",
        vehicle_id: int = 0,
        # Spawning pose of the vehicle
        init_pos=[0.0, 0.0, 0.5],
        init_orientation=[0.0, 0.0, 0.0, 1.0],
        config=TailsitterConfig(),
    ):  
       # 1. Initialize the Vehicle base class
        super().__init__(
            stage_prefix, 
            usd_file, 
            init_pos, 
            init_orientation, 
            config.sensors, 
            config.graphical_sensors, 
            config.graphs, 
            config.backends
        )
        # 2. Store configuration parameters
        self._config = config
        self._vehicle_id = vehicle_id
        self._drag = config.drag
        self._simulation_mode = config.simulation_mode
        

    # ===========================================================
    def update(self, dt:float):
        # Adapted from multirotor.py
        # Handle backend communication and sensor updates for physics simulation
       
        '''force/thrust axis test'''
        self.apply_force([20.0, 0.0, 0.0], body_part = "/body" )
        '''
        Body Force [+x, +y, +z] = world [+x, ?, -x]
        Body Torque [+x, +y, +z] = world [?, ?, ?] r/p/y dir
        '''

        # for backend in self._backends:
        #     backend.update(dt)
        