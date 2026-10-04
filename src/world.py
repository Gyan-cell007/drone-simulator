"""
world.py
=============================================================================
Defines the simulation environment, track path, progress tracking, and landing pads.

For beginners:
- In robotics, the "World" represents the competition course.
- It contains an amber-colored path from a green Start Pad to a red Landing Pad.
- It calculates distance metrics and tracks progress (0% to 100%) as the drone
  flies along the track toward the finish.
=============================================================================
"""

import math
from typing import List, Tuple, Optional, Dict, Any
from config import (
    DEFAULT_TRACK_WAYPOINTS,
    PATH_LINE_WIDTH,
    TARGET_LINE_COLOR,
)


class World:
    """
    Represents the competition course and environment geometry.
    """

    def __init__(
        self,
        waypoints: Optional[List[Tuple[float, float]]] = None,
        distractor_lines: Optional[List[Dict[str, Any]]] = None,
        target_line_color: Tuple[int, int, int] = TARGET_LINE_COLOR,
    ):
        raw_points = waypoints if waypoints is not None else DEFAULT_TRACK_WAYPOINTS
        self.waypoints: List[Tuple[float, float]] = [(float(x), float(y)) for x, y in raw_points]
        self.line_width: float = float(PATH_LINE_WIDTH)
        self.target_line_color = self._normalize_color(target_line_color)
        self.distractor_lines: List[Dict[str, Any]] = []
        for line in distractor_lines or []:
            self.add_distractor_line(
                line["waypoints"], line["color"], line.get("line_width", self.line_width)
            )

        # Start and Finish locations
        self.start_pos = self.waypoints[0]
        self.landing_pos = self.waypoints[-1]
        self.pad_radius = 30.0

        # Calculate initial drone heading along the first track segment
        dx = self.waypoints[1][0] - self.waypoints[0][0]
        dy = self.waypoints[1][1] - self.waypoints[0][1]
        self.initial_heading = math.degrees(math.atan2(dy, dx)) % 360.0

        # Precompute segment lengths and cumulative distance for progress tracking
        self.segment_lengths: List[float] = []
        self.cumulative_lengths: List[float] = [0.0]
        total_len = 0.0

        for i in range(len(self.waypoints) - 1):
            p1 = self.waypoints[i]
            p2 = self.waypoints[i + 1]
            seg_len = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
            self.segment_lengths.append(seg_len)
            total_len += seg_len
            self.cumulative_lengths.append(total_len)

        self.total_path_length = total_len

    @staticmethod
    def _normalize_color(color: Tuple[int, int, int]) -> Tuple[int, int, int]:
        rgb = tuple(int(channel) for channel in color)
        if len(rgb) != 3 or any(channel < 0 or channel > 255 for channel in rgb):
            raise ValueError("Line colors must be RGB triples in the range 0..255.")
        return rgb

    def add_distractor_line(
        self,
        waypoints: List[Tuple[float, float]],
        color: Tuple[int, int, int],
        line_width: Optional[float] = None,
    ) -> None:
        """Add a rendered, non-target line with its own simulated RGB identity."""
        if len(waypoints) < 2:
            raise ValueError("A distractor line needs at least two points.")
        self.distractor_lines.append({
            "waypoints": [(float(x), float(y)) for x, y in waypoints],
            "color": self._normalize_color(color),
            "line_width": self.line_width if line_width is None else float(line_width),
        })

    def point_to_segment_distance(
        self, px: float, py: float, ax: float, ay: float, bx: float, by: float
    ) -> Tuple[float, Tuple[float, float], float]:
        """
        Compute the shortest distance from point P to line segment AB.
        
        Returns:
            (distance, (closest_x, closest_y), projection_parameter_t)
        """
        segment_len_sq = (bx - ax) ** 2 + (by - ay) ** 2
        if segment_len_sq == 0.0:
            return math.hypot(px - ax, py - ay), (ax, ay), 0.0

        # Projection parameter t of point P onto infinite line AB
        # t = ((P - A) . (B - A)) / |B - A|^2
        t = ((px - ax) * (bx - ax) + (py - ay) * (by - ay)) / segment_len_sq
        t_clamped = max(0.0, min(1.0, t))

        closest_x = ax + t_clamped * (bx - ax)
        closest_y = ay + t_clamped * (by - ay)
        dist = math.hypot(px - closest_x, py - closest_y)
        return dist, (closest_x, closest_y), t_clamped

    def query_point_on_path(
        self, px: float, py: float, custom_tolerance: Optional[float] = None
    ) -> Tuple[bool, float, Tuple[float, float]]:
        """
        Check if a given (px, py) coordinate is touching the colored path.
        
        Returns:
            (is_on_path, minimum_distance, closest_point_on_path)
        """
        half_width = (self.line_width / 2.0) if custom_tolerance is None else custom_tolerance
        min_dist = float("inf")
        best_closest = (0.0, 0.0)

        for i in range(len(self.waypoints) - 1):
            ax, ay = self.waypoints[i]
            bx, by = self.waypoints[i + 1]
            dist, closest, _ = self.point_to_segment_distance(px, py, ax, ay, bx, by)
            if dist < min_dist:
                min_dist = dist
                best_closest = closest

        is_on_path = min_dist <= half_width
        return is_on_path, min_dist, best_closest

    def sample_line_color(self, px: float, py: float) -> Optional[Tuple[int, int, int]]:
        """Return the simulated visible RGB color at a point, or None for floor.

        This is a deterministic geometry/color abstraction, not camera or image
        processing. Distractors are considered after the target, matching their
        draw order and allowing them to occlude it where the strokes overlap.
        """
        color = None
        lines = [(self.waypoints, self.target_line_color, self.line_width)]
        lines.extend(
            (line["waypoints"], line["color"], line["line_width"])
            for line in self.distractor_lines
        )
        for points, line_color, width in lines:
            for (ax, ay), (bx, by) in zip(points, points[1:]):
                distance, _, _ = self.point_to_segment_distance(px, py, ax, ay, bx, by)
                if distance <= width / 2.0:
                    color = line_color
        return color

    def calculate_progress(self, drone_x: float, drone_y: float) -> float:
        """
        Calculate the drone's travel progress along the path (0.0% to 100.0%).
        """
        if self.is_at_landing_pad(drone_x, drone_y):
            return 100.0

        min_dist = float("inf")
        best_seg_idx = 0
        best_t = 0.0

        # Find which track segment the drone is closest to
        for i in range(len(self.waypoints) - 1):
            ax, ay = self.waypoints[i]
            bx, by = self.waypoints[i + 1]
            dist, _, t = self.point_to_segment_distance(drone_x, drone_y, ax, ay, bx, by)
            if dist < min_dist:
                min_dist = dist
                best_seg_idx = i
                best_t = t

        # Distance traveled along the track up to this point
        dist_along = self.cumulative_lengths[best_seg_idx] + best_t * self.segment_lengths[best_seg_idx]
        progress_pct = (dist_along / self.total_path_length) * 100.0
        return max(0.0, min(100.0, progress_pct))

    def is_at_landing_pad(self, x: float, y: float) -> bool:
        """Check if the position has reached the landing pad."""
        lx, ly = self.landing_pos
        return math.hypot(x - lx, y - ly) <= self.pad_radius

    def is_over_start_pad(self, x: float, y: float) -> bool:
        """Check if the position is within the start pad circle."""
        sx, sy = self.start_pos
        return math.hypot(x - sx, y - sy) <= self.pad_radius
