"""
PyBullet Simulator HAL Implementation

Lightweight physics-based simulation for rapid testing without hardware.
Uses PyBullet physics engine to simulate hexacopter flight dynamics.
"""

import logging
import time
import numpy as np
from typing import Callable, Dict, List, Tuple
from src.hal.hal import (
    HALPlatform, HALTimer, HALPWMChannel, HALInputCapture, HALI2C
)

# Configure logging
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

# Try to import pybullet
try:
    import pybullet as p
    import pybullet_data
    PYBULLET_AVAILABLE = True
except ImportError:
    logger.warning(
        "PyBullet not installed or incompatible Python version. "
        "PyBullet requires Python ≤ 3.12. "
        "Current Python: Use 'python --version' to check. "
        "See PYBULLET_SETUP.md for installation instructions."
    )
    PYBULLET_AVAILABLE = False
    p = None
    pybullet_data = None


class HexacopterPhysics:
    """Hexacopter physics model for PyBullet"""
    
    # Hexacopter geometry (in meters, from center)
    MOTOR_POSITIONS = [
        np.array([0.25, 0.0, 0.0]),      # Motor 0: Front
        np.array([0.125, 0.217, 0.0]),   # Motor 1: Front-right
        np.array([-0.125, 0.217, 0.0]),  # Motor 2: Rear-right
        np.array([-0.25, 0.0, 0.0]),     # Motor 3: Rear
        np.array([-0.125, -0.217, 0.0]), # Motor 4: Rear-left
        np.array([0.125, -0.217, 0.0]),  # Motor 5: Front-left
    ]
    
    # Motor rotation directions (1=CCW, -1=CW for torque)
    MOTOR_DIRECTIONS = [1, -1, 1, -1, 1, -1]
    
    # Physical constants
    MASS = 1.5  # kg
    GRAVITY = 9.81  # m/s^2
    MAX_THRUST_PER_MOTOR = 15.0  # Newtons
    TORQUE_TO_THRUST_RATIO = 0.05  # Nm per N of thrust
    DRAG_COEFFICIENT = 0.1
    
    def __init__(self, client_id: int):
        self.client = client_id
        self.drone_id = None
        self.motor_thrusts = [0.0] * 6
        
    def create_hexacopter(self, start_pos: List[float]) -> int:
        """Create hexacopter in simulation"""
        if not PYBULLET_AVAILABLE:
            raise RuntimeError("PyBullet not available")
        
        # Create simple box collision shape for body
        collision_shape = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=[0.1, 0.1, 0.05],
            physicsClientId=self.client
        )
        
        # Create visual shape
        visual_shape = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[0.1, 0.1, 0.05],
            rgbaColor=[0.2, 0.2, 0.8, 1.0],
            physicsClientId=self.client
        )
        
        # Create multi-body
        self.drone_id = p.createMultiBody(
            baseMass=self.MASS,
            baseCollisionShapeIndex=collision_shape,
            baseVisualShapeIndex=visual_shape,
            basePosition=start_pos,
            baseOrientation=[0, 0, 0, 1],
            physicsClientId=self.client
        )
        
        # Add motor visual markers (small spheres)
        for i, pos in enumerate(self.MOTOR_POSITIONS):
            motor_vis = p.createVisualShape(
                p.GEOM_SPHERE,
                radius=0.03,
                rgbaColor=[1.0, 0.0, 0.0, 1.0],
                physicsClientId=self.client
            )
            p.createMultiBody(
                baseMass=0,
                baseVisualShapeIndex=motor_vis,
                basePosition=[start_pos[0] + pos[0], start_pos[1] + pos[1], start_pos[2]],
                physicsClientId=self.client
            )
        
        logger.info(f"Created hexacopter at position {start_pos}")
        return self.drone_id
    
    def set_motor_thrust(self, motor_index: int, thrust_percent: float):
        """Set motor thrust (0-100%)"""
        if 0 <= motor_index < 6:
            self.motor_thrusts[motor_index] = thrust_percent / 100.0
            logger.debug(f"Motor {motor_index} thrust: {thrust_percent:.1f}%")
    
    def apply_forces(self):
        """Apply motor forces and torques to drone"""
        if self.drone_id is None:
            return
        
        # Get current orientation
        pos, orn = p.getBasePositionAndOrientation(self.drone_id, physicsClientId=self.client)
        rot_matrix = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)
        
        # Calculate total thrust and torques
        total_thrust = np.array([0.0, 0.0, 0.0])
        total_torque = np.array([0.0, 0.0, 0.0])
        
        for i in range(6):
            # Thrust force (in body frame, pointing up)
            thrust_magnitude = self.motor_thrusts[i] * self.MAX_THRUST_PER_MOTOR
            thrust_body = np.array([0, 0, thrust_magnitude])
            
            # Transform to world frame
            thrust_world = rot_matrix @ thrust_body
            total_thrust += thrust_world
            
            # Torque from motor position (cross product)
            motor_pos_body = self.MOTOR_POSITIONS[i]
            torque_from_position = np.cross(motor_pos_body, thrust_body)
            
            # Torque from motor rotation (reaction torque)
            torque_from_rotation = np.array([0, 0, self.MOTOR_DIRECTIONS[i] * thrust_magnitude * self.TORQUE_TO_THRUST_RATIO])
            
            total_torque += torque_from_position + torque_from_rotation
        
        # Apply forces and torques
        p.applyExternalForce(
            self.drone_id, -1,
            total_thrust.tolist(),
            pos,
            p.WORLD_FRAME,
            physicsClientId=self.client
        )
        
        # Transform torque to world frame and apply
        total_torque_world = rot_matrix @ total_torque
        p.applyExternalTorque(
            self.drone_id, -1,
            total_torque_world.tolist(),
            p.WORLD_FRAME,
            physicsClientId=self.client
        )
        
        # Apply simple drag
        vel, ang_vel = p.getBaseVelocity(self.drone_id, physicsClientId=self.client)
        drag_force = -np.array(vel) * self.DRAG_COEFFICIENT
        p.applyExternalForce(
            self.drone_id, -1,
            drag_force.tolist(),
            pos,
            p.WORLD_FRAME,
            physicsClientId=self.client
        )
    
    def get_state(self) -> Dict:
        """Get current state (position, velocity, orientation, etc)"""
        if self.drone_id is None:
            return {}
        
        pos, orn = p.getBasePositionAndOrientation(self.drone_id, physicsClientId=self.client)
        vel, ang_vel = p.getBaseVelocity(self.drone_id, physicsClientId=self.client)
        
        # Convert quaternion to euler angles
        euler = p.getEulerFromQuaternion(orn)
        
        return {
            'position': np.array(pos),
            'velocity': np.array(vel),
            'orientation_quat': np.array(orn),
            'orientation_euler': np.array(euler),  # [roll, pitch, yaw]
            'angular_velocity': np.array(ang_vel),
        }
    
    def apply_external_force(self, force: List[float], position: List[float] = None):
        """Apply external force (e.g., wind gust)"""
        if self.drone_id is None:
            return
        
        if position is None:
            # Apply at center of mass
            pos, _ = p.getBasePositionAndOrientation(self.drone_id, physicsClientId=self.client)
            position = pos
        
        p.applyExternalForce(
            self.drone_id, -1,
            force,
            position,
            p.WORLD_FRAME,
            physicsClientId=self.client
        )
    
    def reset_position(self, position: List[float], orientation_euler: List[float]):
        """Reset drone position and orientation"""
        if self.drone_id is None:
            return
        
        # Convert euler angles to quaternion
        orn_quat = p.getQuaternionFromEuler(orientation_euler)
        
        p.resetBasePositionAndOrientation(
            self.drone_id,
            position,
            orn_quat,
            physicsClientId=self.client
        )
        
        # Reset velocities
        p.resetBaseVelocity(
            self.drone_id,
            [0, 0, 0],
            [0, 0, 0],
            physicsClientId=self.client
        )


class PyBulletPWMChannel(HALPWMChannel):
    """PyBullet PWM channel (motor control)"""
    
    def __init__(self, physics: HexacopterPhysics, motor_index: int):
        self.physics = physics
        self.motor_index = motor_index
        self._pulse_width = 1000
    
    def set_pulse_width(self, width_us: int):
        """Set PWM pulse width and update motor thrust"""
        self.validate_pulse_width(width_us)
        self._pulse_width = width_us
        
        # Convert PWM (1000-2000us) to thrust (0-100%)
        # 1000us = 0%, 2000us = 100%
        thrust_percent = max(0, min(100, (width_us - 1000) / 10.0))
        self.physics.set_motor_thrust(self.motor_index, thrust_percent)
    
    def get_pulse_width(self) -> int:
        return self._pulse_width


class PyBulletInputCapture(HALInputCapture):
    """PyBullet RC input simulation"""
    
    def __init__(self, channel: int):
        self.channel = channel
        self._pulse_width = 1500
        self._callback = None
    
    def get_pulse_width(self) -> int:
        return self._pulse_width
    
    def set_callback(self, callback: Callable):
        self._callback = callback
    
    def set_simulated_input(self, width_us: int):
        """Set simulated RC input (for testing)"""
        self._pulse_width = width_us
        if self._callback:
            self._callback(width_us)


class PyBulletI2C(HALI2C):
    """PyBullet I2C sensor simulation (IMU)"""
    
    def __init__(self, physics: HexacopterPhysics):
        self.physics = physics
        self._initialized = False
        
        # Sensor noise parameters
        self.accel_noise_std = 0.01  # m/s^2
        self.gyro_noise_std = 0.001  # rad/s
        self.accel_bias = np.array([0.0, 0.0, 0.0])
        self.gyro_bias = np.array([0.0, 0.0, 0.0])
    
    def write_byte(self, device_addr: int, register: int, value: int):
        """Simulated I2C write (sensor configuration)"""
        if device_addr == 0x68 and register == 0x6B:
            # MPU6050 power management - wake up
            self._initialized = True
            logger.debug("IMU initialized")
    
    def read_byte(self, device_addr: int, register: int) -> int:
        """Simulated I2C read"""
        if device_addr == 0x68 and register == 0x75:
            # WHO_AM_I register
            return 0x68  # MPU6050 device ID
        return 0
    
    def read_bytes(self, device_addr: int, register: int, length: int) -> bytes:
        """Read IMU data from physics simulation"""
        if length <= 0:
            raise ValueError(f"Invalid read length: {length}")
        
        if device_addr == 0x68 and register == 0x3B:
            # Accelerometer data (6 bytes: X, Y, Z as int16)
            state = self.physics.get_state()
            
            if not state:
                return bytes(length)
            
            # Get linear acceleration (including gravity)
            vel = state['velocity']
            ang_vel = state['angular_velocity']
            
            # Estimate acceleration (numerical differentiation would be better)
            # For now, use gravity vector transformed to body frame
            orn_quat = state['orientation_quat']
            rot_matrix = np.array(p.getMatrixFromQuaternion(orn_quat)).reshape(3, 3)
            
            # Gravity in world frame
            gravity_world = np.array([0, 0, -self.physics.GRAVITY])
            
            # Transform to body frame (what IMU measures)
            gravity_body = rot_matrix.T @ gravity_world
            
            # Add noise and bias
            accel_measured = gravity_body + self.accel_bias
            accel_measured += np.random.normal(0, self.accel_noise_std, 3)
            
            # Convert to raw sensor values (LSB/g for ±16g range)
            # MPU6050: 2048 LSB/g at ±16g setting
            accel_raw = (accel_measured / self.physics.GRAVITY * 2048).astype(np.int16)
            
            # Pack as bytes (big endian)
            data = []
            for val in accel_raw:
                data.append((val >> 8) & 0xFF)  # MSB
                data.append(val & 0xFF)          # LSB
            
            return bytes(data)
        
        elif device_addr == 0x68 and register == 0x43:
            # Gyroscope data (6 bytes: X, Y, Z as int16)
            state = self.physics.get_state()
            
            if not state:
                return bytes(length)
            
            # Get angular velocity (body frame)
            gyro_measured = state['angular_velocity'] + self.gyro_bias
            gyro_measured += np.random.normal(0, self.gyro_noise_std, 3)
            
            # Convert rad/s to raw (131 LSB per deg/s at ±250deg/s setting)
            gyro_raw = (gyro_measured * 180 / np.pi * 131).astype(np.int16)
            
            # Pack as bytes
            data = []
            for val in gyro_raw:
                data.append((val >> 8) & 0xFF)
                data.append(val & 0xFF)
            
            return bytes(data)
        
        return bytes(length)
    
    def write_bytes(self, device_addr: int, register: int, data: bytes):
        """Simulated I2C write"""
        if not data:
            raise ValueError("Empty data buffer")
        logger.debug(f"I2C write: addr=0x{device_addr:02X} reg=0x{register:02X} len={len(data)}")


class PyBulletTimer(HALTimer):
    """PyBullet timer"""
    
    def __init__(self, timer_id: int, physics: HexacopterPhysics):
        self.timer_id = timer_id
        self.physics = physics
        self.channels = {}
    
    def create_pwm_channel(self, pin: str, frequency: int) -> HALPWMChannel:
        """Create PWM output for motor control"""
        if frequency <= 0:
            raise ValueError(f"Invalid frequency: {frequency}")
        
        # Extract motor index from pin name (e.g., 'X1' -> 0, 'X2' -> 1)
        motor_index = int(pin[1]) - 1 if len(pin) > 1 else 0
        motor_index = max(0, min(5, motor_index))  # Clamp to 0-5
        
        pwm = PyBulletPWMChannel(self.physics, motor_index)
        self.channels[pin] = pwm
        logger.debug(f"Created PWM channel for motor {motor_index} on pin {pin}")
        return pwm
    
    def create_input_capture(self, pin: str, callback: Callable) -> HALInputCapture:
        """Create RC input capture"""
        channel = int(pin[1]) - 1 if len(pin) > 1 else 0
        ic = PyBulletInputCapture(channel)
        ic.set_callback(callback)
        self.channels[pin] = ic
        logger.debug(f"Created input capture on pin {pin}")
        return ic
    
    def deinit(self):
        """Cleanup"""
        self.channels.clear()


class PyBulletPlatform(HALPlatform):
    """PyBullet simulator platform"""
    
    def __init__(self, gui: bool = True, start_position: List[float] = None, start_orientation: List[float] = None):
        """Initialize PyBullet simulation
        
        Args:
            gui: Show GUI window (True) or run headless (False)
            start_position: Starting [x, y, z] position in meters
            start_orientation: Starting [roll, pitch, yaw] orientation in radians
        """
        if not PYBULLET_AVAILABLE:
            raise RuntimeError(
                "PyBullet not installed. Install with: pip install pybullet"
            )
        
        # Connect to PyBullet
        self.client = p.connect(p.GUI if gui else p.DIRECT)
        logger.info(f"PyBullet connected (GUI={gui})")
        
        # Set up simulation
        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=self.client)
        p.setGravity(0, 0, -9.81, physicsClientId=self.client)
        p.setRealTimeSimulation(0, physicsClientId=self.client)  # Manual stepping
        p.setTimeStep(1/240, physicsClientId=self.client)  # 240Hz physics
        
        # Load ground plane
        p.loadURDF("plane.urdf", physicsClientId=self.client)
        
        # Create hexacopter physics
        self.physics = HexacopterPhysics(self.client)
        start_pos = start_position or [0, 0, 1.0]
        self.physics.create_hexacopter(start_pos)
        
        # Apply initial orientation if specified
        if start_orientation is not None:
            self.physics.reset_position(start_pos, start_orientation)
        
        # Camera setup (if GUI)
        if gui:
            p.resetDebugVisualizerCamera(
                cameraDistance=3.0,
                cameraYaw=45,
                cameraPitch=-30,
                cameraTargetPosition=start_pos,
                physicsClientId=self.client
            )
        
        self.timers = {}
        self.i2c_buses = {}
        self._start_time = time.time()
        self._sim_time = 0
        self._step_count = 0
        
        logger.info("PyBullet platform initialized")
    
    def get_timer(self, timer_id: int) -> HALTimer:
        """Get or create timer"""
        if not (1 <= timer_id <= 14):
            raise ValueError(f"Invalid timer ID {timer_id}")
        
        if timer_id not in self.timers:
            self.timers[timer_id] = PyBulletTimer(timer_id, self.physics)
        return self.timers[timer_id]
    
    def get_i2c(self, bus_id: int) -> HALI2C:
        """Get or create I2C bus"""
        if bus_id not in (1, 2):
            raise ValueError(f"Invalid I2C bus ID {bus_id}")
        
        if bus_id not in self.i2c_buses:
            self.i2c_buses[bus_id] = PyBulletI2C(self.physics)
        return self.i2c_buses[bus_id]
    
    def step_simulation(self):
        """Step physics simulation forward one timestep"""
        # Apply motor forces
        self.physics.apply_forces()
        
        # Step physics
        p.stepSimulation(physicsClientId=self.client)
        
        self._step_count += 1
        self._sim_time = self._step_count / 240.0  # 240Hz timestep
    
    def get_simulation_state(self) -> Dict:
        """Get current simulation state"""
        return self.physics.get_state()
    
    def millis(self) -> int:
        """Milliseconds since start (simulation time)"""
        return int(self._sim_time * 1000)
    
    def delay_ms(self, ms: int):
        """Delay milliseconds (simulation time)"""
        steps = int((ms / 1000.0) * 240)  # 240Hz
        for _ in range(steps):
            self.step_simulation()
    
    def delay_us(self, us: int):
        """Delay microseconds (simulation time)"""
        if us >= 1000:
            self.delay_ms(us // 1000)
    
    @property
    def platform_name(self) -> str:
        return "PyBullet"
    
    def cleanup(self):
        """Clean up simulation"""
        if hasattr(self, 'client'):
            p.disconnect(physicsClientId=self.client)
            logger.info("PyBullet disconnected")
