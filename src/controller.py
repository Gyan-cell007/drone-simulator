"""
controller.py
=============================================================================
Drone Navigation & Autopilot Controller (Stage 1).

For beginners:
- A "Controller" is the decision-making brain of the drone.
- It takes the "lateral error" from the camera sensor:
    - error = 0 : drone is centered on the line
    - error > 0 : line is to the right -> turn right (clockwise)
    - error < 0 : line is to the left -> turn left (counter-clockwise)

How our Simple PD Controller works:
1. P (Proportional):
   Steers proportional to current error.
   If you are 20 pixels off, steer twice as hard as when you are 10 pixels off.
   p_term = Kp * error

2. D (Derivative):
   Prevents overshooting (weaving back and forth).
   It measures how fast the error is changing (delta_error = error - prev_error)
   and starts straightening the wheel BEFORE crossing the center line.
   d_term = Kd * (error - prev_error)

3. Throttle (Speed) Modulation:
   - On straights (|error| < 12 px): cruise fast at ~70% throttle.
   - On curves (|error| >= 12 px): automatically slow down to ~50% throttle.
   - At the Landing Pad: cut throttle completely to perform a stationary landing.
=============================================================================
"""

from enum import Enum
import math
from typing import Tuple, Dict, Any
from src.sensors import LineSensor
from src.world import World
from config import (
    KNOWN_PATH_MAX_DISTANCE,
    KNOWN_PATH_SENSOR_CORRECTION_LIMIT,
    KNOWN_PATH_HEADING_GAIN_DEG,
    KNOWN_PATH_LOOKAHEAD_DIST,
    KNOWN_PATH_MIN_SPEED,
    KNOWN_PATH_MAX_SPEED,
    KNOWN_PATH_ACCEL_LIMIT,
    KNOWN_PATH_DECEL_LIMIT,
    KNOWN_PATH_MIN_LOOKAHEAD_DIST,
    KNOWN_PATH_MAX_LOOKAHEAD_DIST,
    AUTOPILOT_CRUISE_SPEED,
    AUTOPILOT_LINE_LOSS_RECOVERY_DURATION,
    AUTOPILOT_LINE_LOSS_RECOVERY_THROTTLE,
)


class NavigationMode(Enum):
    AUTOPILOT = "AUTOPILOT (LINE FOLLOWER)"
    KNOWN_PATH = "KNOWN_PATH (ROUTE + SENSOR CORRECTION)"
    PATH_MAPPING = "PATH_MAPPING (LINE FOLLOWER + RECORD)"
    MANUAL = "MANUAL (PILOT OVERRIDE)"
    LANDED = "LANDED (COURSE COMPLETE)"


class DroneController:
    """
    Computes steering and throttle commands to guide the drone along the line.
    """

    def __init__(self, kp: float = 0.035, kd: float = 0.20):
        # Navigation mode
        self.mode: NavigationMode = NavigationMode.AUTOPILOT

        # Controller gains
        self.kp: float = float(kp)  # Proportional gain
        self.kd: float = float(kd)  # Derivative damping gain

        # Internal state
        self.prev_error: float = 0.0
        self.throttle_cmd: float = 0.0
        self.steering_cmd: float = 0.0

        # Mission status
        self.course_completed: bool = False
        self._line_loss_elapsed: float = 0.0
        self.route_tracker = None
        self.known_path_state = "OFF"
        self.route_distance = 0.0
        self.route_progress = 0.0
        self.lookahead_target = None
        self.sensor_correction = 0.0
        self._known_prev_error = None
        self.known_path_target_speed = 0.0
        self.known_path_speed = 0.0
        self.known_path_turn_class = "STRAIGHT"
        self.adaptive_lookahead_distance = float(KNOWN_PATH_LOOKAHEAD_DIST)
        self.similarity_comparator = None
        self.similarity_score = 0.0
        self.similarity_confidence = 0.0
        self.similarity_decision = "NOT CHECKED"
        self.similarity_samples = 0
        self._route_guidance_active = False

    def reset(self):
        """Reset controller state for a new flight run."""
        self.mode = NavigationMode.AUTOPILOT
        self.prev_error = 0.0
        self.throttle_cmd = 0.0
        self.steering_cmd = 0.0
        self.course_completed = False
        self._line_loss_elapsed = 0.0
        self.clear_known_path()

    def start_known_path(self, route_tracker, similarity_comparator):
        """Activate route guidance with a fresh monotonic route cursor."""
        self.route_tracker = route_tracker
        self.route_tracker.cursor_distance = 0.0
        self.route_tracker.cursor_segment = None
        self.route_tracker.last_projection = None
        self.similarity_comparator = similarity_comparator
        self.mode = NavigationMode.KNOWN_PATH
        self.known_path_state = "ACQUIRING ROUTE"
        self.route_distance = 0.0
        self.route_progress = 0.0
        self.lookahead_target = None
        self.sensor_correction = 0.0
        self._known_prev_error = None
        self.known_path_target_speed = 0.0
        self.known_path_speed = 0.0
        self.known_path_turn_class = "STRAIGHT"
        self.adaptive_lookahead_distance = float(KNOWN_PATH_LOOKAHEAD_DIST)
        self.similarity_score = 0.0
        self.similarity_confidence = 0.0
        snapshot = similarity_comparator.snapshot() if similarity_comparator is not None else None
        self.similarity_decision = snapshot["decision"] if snapshot else "GATHERING EVIDENCE"
        if snapshot:
            self.similarity_score = snapshot["score"]
            self.similarity_confidence = snapshot["confidence"]
            self.similarity_samples = snapshot["samples"]
        else:
            self.similarity_samples = 0
        self._route_guidance_active = False

    def clear_known_path(self, clear_similarity: bool = True):
        self.route_tracker = None
        self.similarity_comparator = None
        self.known_path_state = "OFF"
        self.route_distance = 0.0
        self.route_progress = 0.0
        self.lookahead_target = None
        self.sensor_correction = 0.0
        self._known_prev_error = None
        self.known_path_target_speed = 0.0
        self.known_path_speed = 0.0
        self.known_path_turn_class = "STRAIGHT"
        self.adaptive_lookahead_distance = float(KNOWN_PATH_LOOKAHEAD_DIST)
        self._route_guidance_active = False
        if clear_similarity:
            self.similarity_score = 0.0
            self.similarity_confidence = 0.0
            self.similarity_decision = "NOT CHECKED"
            self.similarity_samples = 0

    def process_manual_input(self, keys: Any) -> Tuple[float, float]:
        """Manual keyboard override controls."""
        import pygame
        throttle = 0.0
        steering = 0.0

        if keys[pygame.K_SPACE]:
            return 0.0, 0.0

        if keys[pygame.K_UP] or keys[pygame.K_w]:
            throttle += 1.0
        if keys[pygame.K_DOWN] or keys[pygame.K_s]:
            throttle -= 0.6

        if keys[pygame.K_LEFT] or keys[pygame.K_a]:
            steering -= 1.0
        if keys[pygame.K_RIGHT] or keys[pygame.K_d]:
            steering += 1.0

        return throttle, steering

    def compute_autonomous_controls(
        self,
        drone_state: Dict[str, Any],
        sensor: LineSensor,
        world: World,
        dt: float = 0.0,
    ) -> Tuple[float, float]:
        """
        Simple, robust line-following algorithm with endpoint landing detection.
        """
        drone_x = drone_state["x"]
        drone_y = drone_state["y"]
        progress = world.calculate_progress(drone_x, drone_y)

        if sensor.line_detected:
            self._line_loss_elapsed = 0.0

        # 1. Destination Check: Stop and land when reaching the Landing Pad
        if world.is_at_landing_pad(drone_x, drone_y) and progress > 85.0:
            self.mode = NavigationMode.LANDED
            self.course_completed = True
            self.throttle_cmd = 0.0
            self.steering_cmd = 0.0
            return 0.0, 0.0

        # 2. Line Following Logic
        if sensor.line_detected:
            error = sensor.lateral_error
            delta_error = error - self.prev_error
            self.prev_error = error

            # Proportional term: steer harder when farther away
            p_term = self.kp * error

            # Derivative term: counteracts rapid error change to damp oscillation
            d_term = self.kd * delta_error

            # Combined steering command clamped to [-1.0, 1.0]
            raw_steering = p_term + d_term
            self.steering_cmd = max(-1.0, min(1.0, raw_steering))

            # Adaptive speed: fast on straights, safe on curves
            abs_err = abs(error)
            if abs_err < 12.0:
                self.throttle_cmd = 0.72   # Fast cruising on straight sections
            elif abs_err < 25.0:
                self.throttle_cmd = 0.52   # Gentle cornering speed
            else:
                self.throttle_cmd = 0.38   # Controlled speed on sharper turns

        else:
            # 3. Briefly search toward the last known line direction, then hold.
            if self.mode in (NavigationMode.AUTOPILOT, NavigationMode.PATH_MAPPING):
                self._line_loss_elapsed += max(0.0, float(dt))
                if self._line_loss_elapsed <= AUTOPILOT_LINE_LOSS_RECOVERY_DURATION:
                    self.steering_cmd = 0.6 if sensor.last_known_error > 0 else -0.6
                    self.throttle_cmd = AUTOPILOT_LINE_LOSS_RECOVERY_THROTTLE
                else:
                    self.steering_cmd = 0.0
                    self.throttle_cmd = 0.0
            else:
                # Keep the existing Known Path acquisition/fallback response.
                self.steering_cmd = 0.6 if sensor.last_known_error > 0 else -0.6
                self.throttle_cmd = 0.30

        return self.throttle_cmd, self.steering_cmd

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    def _ramp_known_path_speed(self, target_speed: float, dt: float) -> float:
        """Move the Known Path speed command toward its target at a bounded rate."""
        target = self._clamp(target_speed, KNOWN_PATH_MIN_SPEED, KNOWN_PATH_MAX_SPEED)
        rate = KNOWN_PATH_ACCEL_LIMIT if target > self.known_path_speed else KNOWN_PATH_DECEL_LIMIT
        max_step = max(0.0, float(dt)) * rate
        delta = self._clamp(target - self.known_path_speed, -max_step, max_step)
        self.known_path_speed = self._clamp(
            self.known_path_speed + delta, 0.0, KNOWN_PATH_MAX_SPEED
        )
        return self.known_path_speed

    def compute_known_path_controls(
        self,
        dt: float,
        drone_state: Dict[str, Any],
        sensor: LineSensor,
        world: World,
    ) -> Tuple[float, float]:
        """Follow a saved route, correcting toward the live line when visible."""
        if self.route_tracker is None:
            self.known_path_state = "NO VALID ROUTE"
            return 0.0, 0.0

        if self.similarity_comparator is not None and sensor.observed_path_point is not None:
            comparison = self.similarity_comparator.add_observation(sensor.observed_path_point)
            self.similarity_score = comparison["score"]
            self.similarity_confidence = comparison["confidence"]
            self.similarity_decision = comparison["decision"]
            self.similarity_samples = comparison["samples"]

        if self.similarity_decision == "CHANGED ROUTE":
            self.mode = NavigationMode.AUTOPILOT
            self.known_path_state = "CHANGED ROUTE - AUTOPILOT"
            self.route_tracker = None
            self.similarity_comparator = None
            self.throttle_cmd, self.steering_cmd = self.compute_autonomous_controls(
                drone_state, sensor, world, dt
            )
            if self.mode != NavigationMode.LANDED:
                self.mode = NavigationMode.AUTOPILOT
                self.known_path_state = "CHANGED ROUTE - AUTOPILOT"
            return self.throttle_cmd, self.steering_cmd

        # Do not hand off to the saved route until enough live observations
        # positively identify it. Uncertain scores keep normal line following.
        if self.similarity_decision != "SIMILAR":
            self.known_path_state = "ACQUIRING"
            self.throttle_cmd, self.steering_cmd = self.compute_autonomous_controls(
                drone_state, sensor, world
            )
            if self.mode != NavigationMode.LANDED:
                self.mode = NavigationMode.KNOWN_PATH
                self.known_path_state = "ACQUIRING"
            return self.throttle_cmd, self.steering_cmd

        x, y = drone_state["x"], drone_state["y"]
        projection = self.route_tracker.project(x, y)
        self.route_distance = projection["distance"]
        self.route_progress = projection["progress"]

        # Similarity is established, but the current position still has to
        # join the route before route-based steering becomes active.
        if not self._route_guidance_active and self.route_distance > KNOWN_PATH_MAX_DISTANCE:
            self.known_path_state = "ACQUIRING ROUTE"
            self.lookahead_target = None
            if sensor.line_detected:
                self.throttle_cmd, self.steering_cmd = self.compute_autonomous_controls(
                    drone_state, sensor, world
                )
                if self.mode != NavigationMode.LANDED:
                    self.mode = NavigationMode.KNOWN_PATH
                    self.known_path_state = "ACQUIRING ROUTE"
            else:
                self.known_path_state = "ACQUIRING ROUTE - LINE LOST"
                self.throttle_cmd, self.steering_cmd = 0.0, 0.0
            return self.throttle_cmd, self.steering_cmd
        if not self._route_guidance_active:
            self.known_path_speed = self._clamp(
                max(0.0, float(drone_state.get("speed", 0.0))),
                0.0,
                KNOWN_PATH_MAX_SPEED,
            )
            self._route_guidance_active = True

        # If tracking strays too far, use line following only while the sensor
        # currently sees the line. Without that reference, stop issuing motion.
        if self.route_distance > KNOWN_PATH_MAX_DISTANCE:
            self.lookahead_target = None
            self._known_prev_error = None
            self.sensor_correction = 0.0
            if sensor.line_detected:
                self.known_path_state = "LINE FOLLOWING FALLBACK"
                self.throttle_cmd, self.steering_cmd = self.compute_autonomous_controls(
                    drone_state, sensor, world
                )
                self.known_path_speed = max(0.0, float(drone_state.get("speed", 0.0)))
                if self.mode != NavigationMode.LANDED:
                    self.mode = NavigationMode.KNOWN_PATH
                return self.throttle_cmd, self.steering_cmd
            self.known_path_state = "ROUTE LOST - HOLD"
            self.known_path_speed = max(0.0, float(drone_state.get("speed", 0.0)))
            self.throttle_cmd, self.steering_cmd = 0.0, 0.0
            return self.throttle_cmd, self.steering_cmd

        self.known_path_state = "TRACKING ROUTE"
        self.known_path_target_speed, self.known_path_turn_class = (
            self.route_tracker.target_speed_for_distance()
        )
        self._ramp_known_path_speed(self.known_path_target_speed, dt)
        self.adaptive_lookahead_distance = self.route_tracker.adaptive_lookahead(
            self.known_path_speed, self.known_path_turn_class
        )
        self.lookahead_target = self.route_tracker.lookahead_target(
            self.adaptive_lookahead_distance
        )

        remaining = self.route_tracker.total_length - self.route_tracker.cursor_distance
        if (
            self.route_progress >= 90.0
            and remaining <= self.adaptive_lookahead_distance
            and world.is_at_landing_pad(x, y)
        ):
            self.mode = NavigationMode.LANDED
            self.course_completed = True
            self.known_path_state = "LANDED"
            self.throttle_cmd, self.steering_cmd = 0.0, 0.0
            return self.throttle_cmd, self.steering_cmd

        target = self.lookahead_target
        desired_yaw = math.degrees(math.atan2(target["y"] - y, target["x"] - x))
        heading_error = (desired_yaw - drone_state["yaw"] + 180.0) % 360.0 - 180.0
        route_steering = self._clamp(
            heading_error / KNOWN_PATH_HEADING_GAIN_DEG, -0.85, 0.85
        )

        self.sensor_correction = 0.0
        if sensor.line_detected:
            error = sensor.lateral_error
            delta_error = 0.0 if self._known_prev_error is None else error - self._known_prev_error
            self._known_prev_error = error
            self.sensor_correction = self._clamp(
                self.kp * error + self.kd * delta_error,
                -KNOWN_PATH_SENSOR_CORRECTION_LIMIT,
                KNOWN_PATH_SENSOR_CORRECTION_LIMIT,
            )
        else:
            self._known_prev_error = None

        self.steering_cmd = self._clamp(
            route_steering + self.sensor_correction, -1.0, 1.0
        )

        # Convert the smoothly limited geometry profile (px/s) to the existing
        # normalized throttle command. Historical mapped speed/time are unused.
        self.throttle_cmd = self._clamp(
            self.known_path_speed / AUTOPILOT_CRUISE_SPEED, 0.0,
            KNOWN_PATH_MAX_SPEED / AUTOPILOT_CRUISE_SPEED,
        )
        return self.throttle_cmd, self.steering_cmd

    def update(
        self,
        dt: float,
        drone_state: Dict[str, Any],
        sensor: LineSensor,
        world: World,
        keys: Any = None,
    ) -> Tuple[float, float]:
        """
        Main update step executed once every simulation frame.
        """
        if self.mode in (NavigationMode.AUTOPILOT, NavigationMode.PATH_MAPPING):
            self.throttle_cmd, self.steering_cmd = self.compute_autonomous_controls(
                drone_state, sensor, world, dt
            )

        elif self.mode == NavigationMode.KNOWN_PATH:
            self.throttle_cmd, self.steering_cmd = self.compute_known_path_controls(
                dt, drone_state, sensor, world
            )

        elif self.mode == NavigationMode.LANDED:
            self.throttle_cmd = 0.0
            self.steering_cmd = 0.0

        elif self.mode == NavigationMode.MANUAL:
            if keys is not None:
                self.throttle_cmd, self.steering_cmd = self.process_manual_input(keys)
            else:
                self.throttle_cmd, self.steering_cmd = 0.0, 0.0

        return self.throttle_cmd, self.steering_cmd
