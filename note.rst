# Setting up source code
1. Clone the repository (isaac-uav)
2. Tell IsaacSim's python where UAV code lives
  - ``cd C:\isaac-uav\extensions``
      ``C:\isaac-sim\python.bat -m pip install --editable pegasus.simulator``
  - or if error, directly inject path in .py files (e.g. example_12 line 23)
3. Launch simulation
  - ``cd C:\isaac-uav``
  - ``C:\isaac-sim\python.bat examples\12_ardupilot_fixedwing.py``

# Fixed-wing UAV specific files
1. USD assets: ``extensions\pegasus\simulator\assets\Robots\fixed_wing`` 
2. FixedWing Class definition: ``extensions\pegasus\simulator\logic\vehicles\fixedwing.py``
3. Simulation example script: ``examples\12_ardupilot_fixedwing.py``
