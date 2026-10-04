"""
sensors.py
=============================================================================
Down-facing camera / optical line-following sensor simulation.

For beginners:
- A real drone carries a camera pointing down at the floor.
- To follow a line, the camera looks at a horizontal slice of pixels
  just ahead of the drone (look-ahead distance).
- If the line appears to the right (+ error), the drone must steer right.
- If the line appears to the left (- error), the drone must steer left.
- If the line is dead center (error = 0.0), the drone flies straight.

This module simulates this camera by placing an array of virtual photodetector
probes across a scan bar ahead of the drone.
=============================================================================
"""

import math
from typing import List, Dict, Any, Optional, Tuple
from config import (
    SENSOR_LOOKAHEAD_DIST,
    SENSOR_SPAN_WIDTH,
    SENSOR_NUM_PROBES,
)
from src.world import World


class LineSensor:
    """
    Simulates a downward-facing camera / optical line detector.
    
    Attributes:
        lookahead (float): Distance ahead of drone center to place the sensor bar.
        span (float): Width of the sensor bar in pixels.
        num_probes (int): Number of sample points across the bar.
    """

    def __init__(
        self,
        lookahead: float = SENSOR_LOOKAHEAD_DIST,
        span: float = SENSOR_SPAN_WIDTH,
        num_probes: int = SENSOR_NUM_PROBES,
        target_color: Optional[Tuple[int, int, int]] = None,
    ):
        self.lookahead = float(lookahead)
        self.span = float(span)
        self.num_probes = max(3, int(num_probes))
        self.target_color = (
            None if target_color is None else tuple(int(channel) for channel in target_color)
        )

        # Sensor readings
        self.line_detected: bool = False
        self.lateral_error: float = 0.0
        self.prev_lateral_error: float = 0.0
        self.error_derivative: float = 0.0
        self.last_known_error: float = 0.0
        self.observed_path_point = None
        self.active_probe_count: int = 0
        self.observed_color = None
        self.probes_data: List[Dict[str, Any]] = []

    def update(self, dt: float, drone_x: float, drone_y: float, drone_yaw_deg: float, world: World):
        """
        Sample the environment and calculate lateral tracking error.
        
        Args:
            dt: Frame delta time in seconds.
            drone_x, drone_y: Drone position in world space.
            drone_yaw_deg: Drone heading in degrees.
            world: World object to query path geometry.
        """
        yaw_rad = math.radians(drone_yaw_deg)

        # Unit vector pointing in the drone's forward direction
        forward_x = math.cos(yaw_rad)
        forward_y = math.sin(yaw_rad)

        # Unit vector pointing to the drone's right side (perpendicular to forward)
        # In screen coordinates where Y points down, rotating (cos, sin) 90 deg clockwise gives (-sin, cos)
        right_x = -forward_y
        right_y = forward_x

        # Center point of the sensor scan bar (placed ahead of the drone)
        center_x = drone_x + forward_x * self.lookahead
        center_y = drone_y + forward_y * self.lookahead

        # Lateral offsets spanning from -span/2 (far left) to +span/2 (far right)
        half_span = self.span / 2.0
        step = self.span / (self.num_probes - 1)

        self.probes_data.clear()
        detected_offsets: List[float] = []

        for i in range(self.num_probes):
            lateral_offset = -half_span + i * step  # Negative = Left, Positive = Right
            
            # World position of this specific probe point
            probe_x = center_x + right_x * lateral_offset
            probe_y = center_y + right_y * lateral_offset

            # Query the simulator's color sample; this is not image processing.
            sampled_color = world.sample_line_color(probe_x, probe_y)
            target_color = world.target_line_color if self.target_color is None else self.target_color
            is_on_path = sampled_color == target_color

            if is_on_path:
                detected_offsets.append(lateral_offset)

            self.probes_data.append({
                "x": probe_x,
                "y": probe_y,
                "offset": lateral_offset,
                "detected": is_on_path,
                "color": sampled_color,
            })

        # Calculate tracking outcome
        self.active_probe_count = len(detected_offsets)
        target_color = world.target_line_color if self.target_color is None else self.target_color
        self.observed_color = target_color if self.active_probe_count else None
        if self.active_probe_count > 0:
            self.line_detected = True
            # Centroid is the average lateral position of all active probes
            current_error = sum(detected_offsets) / self.active_probe_count
            self.lateral_error = current_error
            self.last_known_error = current_error
            self.observed_path_point = (
                center_x + right_x * current_error,
                center_y + right_y * current_error,
            )

            # Rate of change of error (derivative)
            if dt > 0.001:
                self.error_derivative = (self.lateral_error - self.prev_lateral_error) / dt
            self.prev_lateral_error = self.lateral_error

        else:
            self.line_detected = False
            self.observed_path_point = None
            # When line is briefly lost, retain sign of last known error to guide recovery
            self.lateral_error = self.last_known_error
            self.error_derivative = 0.0

    def get_readings(self) -> dict:
        """Return the current sensor measurement readings."""
        return {
            "detected": self.line_detected,
            "lateral_error": self.lateral_error,
            "observed_path_point": self.observed_path_point,
            "observed_color": self.observed_color,
            "error_derivative": self.error_derivative,
            "active_probes": self.active_probe_count,
            "total_probes": self.num_probes,
            "probes": self.probes_data,
        }
