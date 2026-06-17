# Setting up source code
1. Clone the repository (isaac-uav)
2. Tell IsaacSim's python where UAV code lives
  - ``cd C:\isaac-uav\extensions``
      ``C:\isaac-sim\python.bat -m pip install --editable pegasus.simulator``
  - or if error, directly inject path in .py files before Pegasus imports (e.g. example_12 line 23)

3. Adapted backend config in 12_ardupilot_fixedwing.py line 102
    - Consideration: 
        - Default ArduPilot+MissionPlanner SITL + now external Flight Dynamics Model (IsaacSim)
        - ArduPilot must run inside Cygwin + MAVProxy network bridge

    - Orignal pipeline: IsaacSim FixedWing -> ArdupilotMavlinkBackend -> ArduP SITL/MissionPlanner
    - Adapted pipeline: 

4. Launch simulation
  - ``cd C:\isaac-uav``
  - ``C:\isaac-sim\python.bat examples\12_ardupilot_fixedwing.py``
  - or ``C:\isaac-uav\app\isaac-sim.bat --ext-folder C:\isaac-uav\extensions``

  - if crashes, 
    - Command Prompt (CMD):
        ``` set ROS_DISTRO=humble
        set RMW_IMPLEMENTATION=rmw_fastrtps_cpp
        set PATH=%PATH%;c:/isaac-sim/exts/isaacsim.ros2.bridge/humble/lib
        ```

# Fixed-wing UAV specific files
1. USD assets: ``extensions\pegasus\simulator\assets\Robots\fixed_wing`` 
2. FixedWing Class definition: ``extensions\pegasus\simulator\logic\vehicles\fixedwing.py``
3. Simulation example script: ``examples\12_ardupilot_fixedwing.py``
4. ROBOTS list: ``extensions\pegasus.simulator\pegasus\simulator\params.py``
  ``ROBOTS = {"Iris": ROBOTS_ASSETS + "/Iris/iris.usd",
          "Fixed Wing": ROBOTS_ASSETS + "/fixed_wing/fixed_wing.usd",
          "Flying Cube": ROBOTS_ASSETS + "/yoda_fixed_wing/cube.usd",}``

---
## Legacy: Setting up PX4 SITL
1. Install PX4 in Ubuntu, configure ip address
2. skip this: (if needed: Update line 219 ``extensions\pegasus\simulator\logic\backends\px4_mavlink_backend.py`` with correct IP and port for MAVLink connection
    e.g. ``self.connection_ip = self.config.get("connection_ip", "172.25.145.97")``)
3. Launch PX4 on WSL2 using 
    ``cd ~/isaac_sim_project/PX4-Autopilot
      make px4_sitl_default none_iris``   
   QGroundControl autoconnects. 
   Then spawn simulation on Pegasus.