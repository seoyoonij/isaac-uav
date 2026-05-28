# Setting up source code
1. Clone the repository (isaac-uav)
2. Tell IsaacSim's python where UAV code lives
  - ``cd C:\isaac-uav\extensions``
      ``C:\isaac-sim\python.bat -m pip install --editable pegasus.simulator``
  - or if error, directly inject path in .py files before Pegasus imports (e.g. example_12 line 23)
3. Launch simulation
  - ``cd C:\isaac-uav``
  - ``C:\isaac-sim\python.bat examples\12_ardupilot_fixedwing.py``

# Fixed-wing UAV specific files
1. USD assets: ``extensions\pegasus\simulator\assets\Robots\fixed_wing`` 
2. FixedWing Class definition: ``extensions\pegasus\simulator\logic\vehicles\fixedwing.py``
3. Simulation example script: ``examples\12_ardupilot_fixedwing.py``
4. ROBOTS list: ``extensions\pegasus.simulator\pegasus\simulator\params.py``
  ``ROBOTS = {"Iris": ROBOTS_ASSETS + "/Iris/iris.usd",
          "Fixed Wing": ROBOTS_ASSETS + "/fixed_wing/fixed_wing.usd",
          "Flying Cube": ROBOTS_ASSETS + "/yoda_fixed_wing/cube.usd",}``
          
# TailsitterConfig parameters to add:
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


# Tailsiter update(): Main control loop
  # ? Update state estimate from simulator
  commands = self.get_commands_from_backend() # Read backend commands
  regime = self.compute_flight_regime() # Decide regime: hover/transition/cruise
  rotor_ft = self.compute_rotor_forces_and_moments(commands, dt) # Compute forces from propellers
  aero_ft = self.compute_aero_forces_and_moments(commands, dt) # Compute aerodynamic forces
  total_ft = self.blend_forces_and_moments(rotor_ft, aero_ft, regime) # Blend forces based on regime and transition logic
  self.apply_total_forces_and_moments(total_ft) # Apply to physics engine
  self.update_actuator_visuals(commands) # Animate