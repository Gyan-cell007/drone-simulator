"""
drone.py
=============================================================================
Defines the Drone physical state, kinematic movement, and telemetry.

For beginners:
- A drone in a 2D top-down simulation has a position (x, y) and an orientation
  (yaw angle in degrees).
- It also has a forward speed (pixels per second) and an angular velocity.
- The physics model here is a kinematic velocity controller:
  throttle smoothly guides forward cruise speed, and steering smoothly rotates
  heading without sudden jerky jumps.
=============================================================================
"""

import math
from config import (
    MAX_SPEED,
    MAX_TURN_RATE,
    DRONE_RADIUS,
    AUTOPILOT_CRUISE_SPEED,
)


class Drone:
    """
    Simulated 2D quadcopter drone.
    
    Attributes:
        x (float): Horizontal position in pixels (0 is left).
        y (float): Vertical position in pixels (0 is top).
        yaw (float): Heading angle in degrees (0 = East, 90 = South, 180 = West, 270 = North).
        speed (float): Current forward velocity (pixels / second).
        altitude (float): Simulated height above ground (0.0 = grounded, 1.0 = hovering).
        rotor_angle (float): Visual rotation angle for spinning propeller animation.
    """

    def __init__(self, start_x: float = 120.0, start_y: float = 560.0, start_yaw: float = 0.0):
        self.start_x = float(start_x)
        self.start_y = float(start_y)
        self.start_yaw = float(start_yaw)

        # Current state
        self.x = self.start_x
        self.y = self.start_y
        self.yaw = self.start_yaw
        self.speed = 0.0
        self.altitude = 1.0  # Cruise hovering altitude in Stage 1

        # Control inputs (-1.0 to 1.0)
        self.throttle_cmd = 0.0
        self.steering_cmd = 0.0

        # Physical limits
        self.max_cruise_speed = float(AUTOPILOT_CRUISE_SPEED)
        self.max_turn_rate = float(MAX_TURN_RATE)
        self.radius = float(DRONE_RADIUS)

        # Visual animation for propellers
        self.rotor_angle = 0.0

    def reset(self, x: float = None, y: float = None, yaw: float = None):
        """Reset the drone back to initial conditions or specified coordinates."""
        self.x = self.start_x if x is None else float(x)
        self.y = self.start_y if y is None else float(y)
        self.yaw = self.start_yaw if yaw is None else float(yaw)
        self.speed = 0.0
        self.altitude = 1.0
        self.throttle_cmd = 0.0
        self.steering_cmd = 0.0

    def set_inputs(self, throttle: float, steering: float):
        """
        Receive control commands from either human input or the autopilot.
        
        Args:
            throttle: Value between -1.0 (reverse / brake) and +1.0 (full forward).
            steering: Value between -1.0 (turn left) and +1.0 (turn right).
        """
        self.throttle_cmd = max(-1.0, min(1.0, float(throttle)))
        self.steering_cmd = max(-1.0, min(1.0, float(steering)))

    def update(self, dt: float):
        """
        Advance the drone physics by dt seconds.
        """
        # 1. Update Yaw (Orientation)
        # Positive steering turns clockwise; negative turns counter-clockwise.
        yaw_rate = self.steering_cmd * self.max_turn_rate
        self.yaw = (self.yaw + yaw_rate * dt) % 360.0

        # 2. Update Velocity
        # Smoothly track target velocity based on throttle
        target_speed = self.throttle_cmd * self.max_cruise_speed
        accel_rate = 6.0  # Response responsiveness
        self.speed += (target_speed - self.speed) * min(1.0, accel_rate * dt)

        # Clamp speed to physical safety bounds
        self.speed = max(-MAX_SPEED * 0.3, min(MAX_SPEED, self.speed))

        # 3. Update Position (x, y)
        yaw_rad = math.radians(self.yaw)
        self.x += self.speed * math.cos(yaw_rad) * dt
        self.y += self.speed * math.sin(yaw_rad) * dt

        # 4. Animate spinning rotor blades
        spin_rate = 900.0 + abs(self.speed) * 6.0
        self.rotor_angle = (self.rotor_angle + spin_rate * dt) % 360.0

    def get_state(self) -> dict:
        """Return a snapshot of current telemetry data."""
        return {
            "x": self.x,
            "y": self.y,
            "yaw": self.yaw,
            "speed": self.speed,
            "altitude": self.altitude,
            "throttle": self.throttle_cmd,
            "steering": self.steering_cmd,
        }
