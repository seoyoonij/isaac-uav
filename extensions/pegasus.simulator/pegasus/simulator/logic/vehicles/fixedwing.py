"""
| File: fixedwing.py
| Author: [Your Name]
| License: BSD-3-Clause
| Description: Definition of the FixedWing class which is used as the base for all fixed-wing aircraft.
"""

import numpy as np
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


class FixedWingConfig:
    """
    A data class that is used for configuring a Fixed-Wing Aircraft
    """

    def __init__(self):
        """
        Initialization of the FixedWingConfig class
        """

        # Stage prefix of the vehicle when spawning in the world
        self.stage_prefix = "fixedwing"

        # The USD file that describes the visual aspect of the vehicle
        self.usd_file = "../../assets/Robots/yoda_fixed_wing/yoda_fixed_wing.usd"

        # ============================================
        # PROPELLER/MOTOR CONFIGURATION
        # ============================================
        # Pervane özellikleri
        self.prop_max_thrust = 50.0  # Maximum thrust in Newtons
        self.prop_max_rpm = 8000.0   # Maximum RPM
        self.prop_thrust_coefficient = 0.00001  # Thrust = coef * RPM^2
        
        # Motor rotation direction (1: CCW, -1: CW when viewed from behind)
        self.prop_rotation_dir = 1

        # ============================================
        # AERODYNAMIC COEFFICIENTS
        # ============================================
        # Lift coefficients
        self.CL_0 = 0.28       # Zero angle of attack lift coefficient
        self.CL_alpha = 3.45   # Lift curve slope (per radian)
        self.CL_max = 1.4      # Maximum lift coefficient (stall limit)
        self.CL_min = -1.1     # Minimum lift coefficient
        
        # Drag coefficients
        self.CD_0 = 0.03       # Zero-lift drag coefficient (parasitic drag)
        self.CD_alpha = 0.30   # Drag due to angle of attack
        self.CD_alpha2 = 2.0   # Induced drag factor (quadratic term)
        
        # Side force coefficients
        self.CY_beta = -0.98   # Side force due to sideslip
        
        # Pitch moment coefficients
        self.Cm_0 = -0.02      # Zero angle of attack pitch moment
        self.Cm_alpha = -0.38  # Pitch moment curve slope
        
        # Roll moment coefficients  
        self.Cl_beta = -0.12   # Roll moment due to sideslip (dihedral effect)
        
        # Yaw moment coefficients
        self.Cn_beta = 0.25    # Yaw moment due to sideslip (weathercock stability)

        # ============================================
        # CONTROL SURFACE DERIVATIVES
        # ============================================
        # Elevator effectiveness (pitch control)
        self.CL_elevator = 0.43    # Lift change per elevator deflection
        self.Cm_elevator = -1.122  # Pitch moment per elevator deflection
        
        # Aileron effectiveness (roll control)
        self.Cl_aileron = 0.229    # Roll moment per aileron deflection
        
        # Rudder effectiveness (yaw control)
        self.Cn_rudder = -0.032    # Yaw moment per rudder deflection
        self.CY_rudder = 0.870     # Side force per rudder deflection

        # ============================================
        # AIRCRAFT GEOMETRY
        # ============================================
        self.wing_area = 0.55          # Wing area in m^2
        self.wing_span = 2.8           # Wing span in m
        self.chord = 0.18880           # Mean aerodynamic chord in m
        self.air_density = 1.225       # Air density in kg/m^3 (sea level)

        # ============================================
        # DRAG CONFIGURATION
        # ============================================
        # Linear drag (for body frame drag effects)
        self.drag = LinearDrag([0.1, 0.1, 0.1])

        # ============================================
        # SENSORS
        # ============================================
        self.sensors = [Barometer(), IMU(), Magnetometer(), GPS()]
        self.graphical_sensors = []
        self.graphs = []

        # ============================================
        # BACKENDS
        # ============================================
        # Control backend (Mavlink, ROS2, or custom)
        self.backends = []


class FixedWing(Vehicle):
    """
    FixedWing class - Defines a base interface for creating fixed-wing aircraft
    """
    
    def __init__(
        self,
        # Simulation specific configurations
        stage_prefix: str = "fixedwing",
        usd_file: str = "",
        vehicle_id: int = 0,
        # Spawning pose of the vehicle
        init_pos=[0.0, 0.0, 0.5],
        init_orientation=[0.0, 0.0, 0.0, 1.0],
        config=FixedWingConfig(),
    ):
        """
        Initializes the fixed-wing aircraft object

        Args:
            stage_prefix (str): The name the vehicle will present in the simulator. Defaults to "fixedwing".
            usd_file (str): The USD file that describes the looks and shape of the vehicle. Defaults to "".
            vehicle_id (int): The id to be used for the vehicle. Defaults to 0.
            init_pos (list): Initial position in inertial frame (ENU). Defaults to [0.0, 0.0, 0.5].
            init_orientation (list): Initial orientation quaternion [qx, qy, qz, qw]. Defaults to [0.0, 0.0, 0.0, 1.0].
            config (FixedWingConfig): Configuration object. Defaults to FixedWingConfig().
        """

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
        
        # Propeller configuration
        self._prop_max_thrust = config.prop_max_thrust
        self._prop_max_rpm = config.prop_max_rpm
        self._prop_thrust_coef = config.prop_thrust_coefficient
        self._prop_rotation_dir = config.prop_rotation_dir
        
        # Aerodynamic coefficients
        self._CL_0 = config.CL_0
        self._CL_alpha = config.CL_alpha
        self._CL_max = config.CL_max
        self._CL_min = config.CL_min
        
        self._CD_0 = config.CD_0
        self._CD_alpha = config.CD_alpha
        self._CD_alpha2 = config.CD_alpha2
        
        self._CY_beta = config.CY_beta
        
        self._Cm_0 = config.Cm_0
        self._Cm_alpha = config.Cm_alpha
        
        self._Cl_beta = config.Cl_beta
        self._Cn_beta = config.Cn_beta
        
        # Control surface derivatives
        self._CL_elevator = config.CL_elevator
        self._Cm_elevator = config.Cm_elevator
        self._Cl_aileron = config.Cl_aileron
        self._Cn_rudder = config.Cn_rudder
        self._CY_rudder = config.CY_rudder
        
        # Aircraft geometry
        self._wing_area = config.wing_area
        self._wing_span = config.wing_span
        self._chord = config.chord
        self._air_density = config.air_density
        
        # Drag model
        self._drag = config.drag
        
        # Current control inputs (will be updated from backend)
        self._throttle = 0.0      # 0.0 to 1.0
        self._elevator = 0.0      # -1.0 to 1.0 (radians or normalized)
        self._aileron = 0.0       # -1.0 to 1.0
        self._rudder = 0.0        # -1.0 to 1.0

    def start(self):
        """
        Called when simulation starts
        """
        pass

    def stop(self):
        """
        Called when simulation stops
        """
        pass

    def update(self, dt: float):
        """
        Main update loop - computes and applies aerodynamic forces and moments
        This is called at every physics step.

        Args:
            dt (float): Time step in seconds
        """
        if not self._prim.IsValid():
            return

        
        # 1. Get control inputs from backend
        self._update_control_inputs()
        
        # 2. Calculate propeller thrust
        thrust_force = self._calculate_propeller_thrust()
        
        # 3. Calculate aerodynamic forces and moments
        aero_forces, aero_moments = self._calculate_aerodynamics()
        
        # 4. Apply propeller thrust (in body frame, pointing forward)
        self.apply_force([thrust_force, 0.0, 0.0], body_part="/body")
        
        # 5. Apply aerodynamic forces
        self.apply_force(aero_forces, body_part="/body")
        
        # 6. Apply aerodynamic moments
        self.apply_torque(aero_moments, body_part="/body")
        
        # 7. Apply drag
        drag_force = self._drag.update(self._state, dt)
        self.apply_force(drag_force, body_part="/body")
        
        # 8. Update propeller visual (if you have a revolute joint named "propeller" or "joint0")
        # self._update_propeller_visual()
        
        # 9. Update backends
        for backend in self._backends:
            backend.update(dt)
            

    def _update_control_inputs(self):
        """
        Updates control surface positions from the backend
        BURADA KENDİ BACKEND YAPISINA GÖRE DÜZENLEME YAPMANIZ GEREKEBİLİR
        """
        if len(self._backends) != 0:
            # Backend'den kontrol komutlarını al
            # PX4 Mavlink kullanıyorsanız, bu format farklı olabilir
            # Örnek: servo output'larını veya PWM değerlerini alın
            
            # ÖNEMLİ: Bu kısmı kendi backend yapınıza göre düzenlemeniz gerekecek
            # Şu an genel bir örnek:
            control_inputs = self._backends[0].input_reference()
            
            # Control inputs format'ı backend'e göre değişir
            # Örnek mapping (kendi sisteminize göre ayarlayın):
            if isinstance(control_inputs, list) and len(control_inputs) >= 4:
                self._throttle = np.clip(control_inputs[0], 0.0, 1.0)
                self._elevator = np.clip(control_inputs[1], -1.0, 1.0)
                self._aileron = np.clip(control_inputs[2], -1.0, 1.0)
                self._rudder = np.clip(control_inputs[3], -1.0, 1.0)
        else:
            # Backend yoksa, sıfır input
            self._throttle = 0.0
            self._elevator = 0.0
            self._aileron = 0.0
            self._rudder = 0.0
        # Eğer backend'iniz dictionary dönüyorsa:
        # ...
        '''
        # Eğer PWM değerleri geliyorsa (1000-2000):
        pwm_values = self._backends[0].input_reference()
        self._throttle = (pwm_values[0] - 1000) / 1000.0  # Normalize to 0-1
        self._elevator = (pwm_values[1] - 1500) / 500.0   # Normalize to -1 to 1
        # ...
        '''
    def _calculate_propeller_thrust(self) -> float:
        """
        Calculates thrust force from propeller based on throttle input
        
        Returns:
            float: Thrust force in Newtons (body frame X-axis)
        """
        # RPM from throttle
        rpm = self._throttle * self._prop_max_rpm
        
        # Thrust = coefficient * RPM^2
        thrust = self._prop_thrust_coef * (rpm ** 2)
        
        # Limit to maximum thrust
        thrust = np.clip(thrust, 0.0, self._prop_max_thrust)
        
        return thrust

    def _calculate_aerodynamics(self):
        """
        Calculates aerodynamic forces and moments based on current flight state
        
        Returns:
            tuple: (forces, moments) in body frame
                forces: [Fx, Fy, Fz] in Newtons
                moments: [Mx, My, Mz] in Newton-meters
        """
        
        # Get airspeed in body frame [u, v, w]
        V_body = self._state.linear_body_velocity
        u, v, w = V_body[0], V_body[1], V_body[2]
        
        # Total airspeed
        V = np.linalg.norm(V_body)
        
        # Avoid division by zero
        if V < 0.1:
            return np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0])
        
        # Calculate angle of attack (alpha) and sideslip (beta)
        alpha = np.arctan2(w, u)  # radians
        beta = np.arcsin(np.clip(v / V, -1.0, 1.0))  # radians
        
        # Dynamic pressure
        q = 0.5 * self._air_density * V**2
        
        # ==========================================
        # LIFT COEFFICIENT
        # ==========================================
        CL = self._CL_0 + self._CL_alpha * alpha + self._CL_elevator * self._elevator
        CL = np.clip(CL, self._CL_min, self._CL_max)  # Stall limits
        
        # ==========================================
        # DRAG COEFFICIENT
        # ==========================================
        CD = self._CD_0 + self._CD_alpha * abs(alpha) + self._CD_alpha2 * alpha**2
        
        # ==========================================
        # SIDE FORCE COEFFICIENT
        # ==========================================
        CY = self._CY_beta * beta + self._CY_rudder * self._rudder
        
        # ==========================================
        # MOMENT COEFFICIENTS
        # ==========================================
        # Roll moment (around X-axis)
        Cl = self._Cl_beta * beta + self._Cl_aileron * self._aileron
        
        # Pitch moment (around Y-axis)
        Cm = self._Cm_0 + self._Cm_alpha * alpha + self._Cm_elevator * self._elevator
        
        # Yaw moment (around Z-axis)
        Cn = self._Cn_beta * beta + self._Cn_rudder * self._rudder
        
        # ==========================================
        # FORCES (Stability frame -> Body frame)
        # ==========================================
        # In stability frame:
        # -D (drag) along velocity
        # Y (side force) 
        # -L (lift) perpendicular to velocity
        
        L = CL * q * self._wing_area
        D = CD * q * self._wing_area
        Y = CY * q * self._wing_area
        
        # Transform from stability to body frame
        # Stability frame is rotated by alpha around Y-axis
        cos_alpha = np.cos(alpha)
        sin_alpha = np.sin(alpha)
        
        Fx = -D * cos_alpha + L * sin_alpha
        Fy = Y
        Fz = -D * sin_alpha - L * cos_alpha
        
        forces = np.array([Fx, Fy, Fz])
        
        # ==========================================
        # MOMENTS
        # ==========================================
        Mx = Cl * q * self._wing_area * self._wing_span
        My = Cm * q * self._wing_area * self._chord
        Mz = Cn * q * self._wing_area * self._wing_span
        
        moments = np.array([Mx, My, Mz])
        
        return forces, moments

    def _update_propeller_visual(self):
        """
        Updates the propeller joint velocity for visual animation
        Pervane görselini döndürmek için joint hızını ayarlar
        
        USD dosyanızda pervanenin bağlı olduğu joint'in adını bulup buraya yazmanız gerekecek
        Örnek: "propeller_joint", "joint0", "prop_joint" vb.
        """
        pass
        # try:
        #     articulation = self.get_dc_interface().get_articulation(self._stage_prefix)
            
        #     # ÖNEMLİ: USD dosyanızdaki pervane joint'inin ismini buraya yazın
        #     # Örnek joint isimleri: "propeller_joint", "joint0", "prop_joint"
        #     propeller_joint_name = "/body/servo_001"  # BUNU KENDİ USD DOSYANIZA GÖRE DEĞİŞTİRİN
            
        #     joint = self.get_dc_interface().find_articulation_dof(articulation, propeller_joint_name)
            
        #     if joint is not None:
        #         # RPM'den angular velocity'ye (rad/s)
        #         rpm = self._throttle * self._prop_max_rpm
        #         angular_vel = (rpm * 2 * np.pi / 60.0) * self._prop_rotation_dir
                
        #         self.get_dc_interface().set_dof_velocity(joint, angular_vel)
        # except Exception as e:
        #     # Joint bulunamazsa veya hata olursa sessizce geç
        #     pass

    def set_control_inputs(self, throttle=None, elevator=None, aileron=None, rudder=None):
        """
        Manuel olarak kontrol girişlerini ayarlamak için yardımcı fonksiyon
        (Test amaçlı kullanılabilir)
        
        Args:
            throttle (float): 0.0 to 1.0
            elevator (float): -1.0 to 1.0
            aileron (float): -1.0 to 1.0
            rudder (float): -1.0 to 1.0
        """
        if throttle is not None:
            self._throttle = np.clip(throttle, 0.0, 1.0)
        if elevator is not None:
            self._elevator = np.clip(elevator, -1.0, 1.0)
        if aileron is not None:
            self._aileron = np.clip(aileron, -1.0, 1.0)
        if rudder is not None:
            self._rudder = np.clip(rudder, -1.0, 1.0)

    def get_aerodynamic_state(self):
        """
        Returns current aerodynamic state information
        Debugging ve analiz için kullanışlı
        
        Returns:
            dict: Dictionary containing aerodynamic parameters
        """
        V_body = self._state.linear_body_velocity
        u, v, w = V_body[0], V_body[1], V_body[2]
        V = np.linalg.norm(V_body)
        
        alpha = np.arctan2(w, u) if V > 0.1 else 0.0
        beta = np.arcsin(np.clip(v / V, -1.0, 1.0)) if V > 0.1 else 0.0
        
        return {
            'airspeed': V,
            'angle_of_attack': np.degrees(alpha),
            'sideslip': np.degrees(beta),
            'throttle': self._throttle,
            'elevator': self._elevator,
            'aileron': self._aileron,
            'rudder': self._rudder
        }
