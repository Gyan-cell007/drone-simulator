"""
path_manager.py
=============================================================================
Autonomous Path Mapping and Route Storage Manager (Stage 2).

For beginners:
- In autonomous navigation, "Path Mapping" means recording where the robot
  actually flew so we can inspect it, replay it, or check if future paths match.
- Saving every single 60 FPS frame would waste memory and produce bloated files.
- This module implements an intelligent, compact sampling algorithm:
  it records a new waypoint ONLY when:
    1. The drone moves a minimum distance (e.g. >= 16 pixels), OR
    2. Its heading turns significantly (e.g. >= 7 degrees), OR
    3. A minimum duration elapses (e.g. >= 0.45 seconds), OR
    4. The drone touches down at the Landing Pad.
- The recorded route is stored in a clean JSON format ('saved_path.json').
=============================================================================
"""

import json
import math
import os
import time
from bisect import bisect_right
from typing import List, Dict, Any, Optional, Tuple

from config import (
    SAVED_PATH_FILE,
    MAP_SAMPLE_MIN_DIST,
    MAP_SAMPLE_MIN_YAW_DEG,
    MAP_SAMPLE_MIN_TIME_SEC,
    KNOWN_PATH_LOOKAHEAD_DIST,
    KNOWN_PATH_SEARCH_AHEAD_SEGMENTS,
    KNOWN_PATH_MIN_SPEED,
    KNOWN_PATH_MAX_SPEED,
    KNOWN_PATH_STRAIGHT_SPEED,
    KNOWN_PATH_GENTLE_CURVE_SPEED,
    KNOWN_PATH_SHARP_TURN_SPEED,
    KNOWN_PATH_GENTLE_TURN_THRESHOLD_DEG,
    KNOWN_PATH_SHARP_TURN_THRESHOLD_DEG,
    KNOWN_PATH_SHARP_TURN_CLUSTER_DIST,
    KNOWN_PATH_SHARP_TURN_MIN_VERTICES,
    KNOWN_PATH_SPEED_LOOKAHEAD_DIST,
    KNOWN_PATH_MIN_LOOKAHEAD_DIST,
    KNOWN_PATH_MAX_LOOKAHEAD_DIST,
    KNOWN_PATH_GENTLE_LOOKAHEAD_SCALE,
    KNOWN_PATH_SHARP_LOOKAHEAD_SCALE,
    ROUTE_SIMILARITY_ACCEPT_THRESHOLD,
    ROUTE_SIMILARITY_UNCERTAIN_THRESHOLD,
    ROUTE_SIMILARITY_CHANGED_THRESHOLD,
    ROUTE_SIMILARITY_MIN_SAMPLES,
    ROUTE_SIMILARITY_MIN_ROUTE_COVERAGE,
    ROUTE_SIMILARITY_SAMPLE_DISTANCE,
    ROUTE_SIMILARITY_POSITION_TOLERANCE,
    ROUTE_SIMILARITY_POSITION_SCALE,
    ROUTE_SIMILARITY_HEADING_SCALE_DEG,
    ROUTE_SIMILARITY_ANCHOR_DISTANCE,
    ROUTE_SIMILARITY_ANCHOR_TIMEOUT_DISTANCE,
)


class PathMapper:
    """
    Manages real-time path recording, route compaction, and JSON file I/O.
    """

    def __init__(self, filename: str = SAVED_PATH_FILE):
        self.filename = filename
        self.status = "OFF"  # "OFF", "RECORDING", "SAVED"
        self.waypoints: List[Dict[str, Any]] = []

        # Sampling thresholds
        self.min_dist = float(MAP_SAMPLE_MIN_DIST)
        self.min_yaw_diff = float(MAP_SAMPLE_MIN_YAW_DEG)
        self.min_time_diff = float(MAP_SAMPLE_MIN_TIME_SEC)

        # Internal trackers
        self.last_x: Optional[float] = None
        self.last_y: Optional[float] = None
        self.last_yaw: Optional[float] = None
        self.last_time: float = 0.0

    def start_recording(self):
        """Begin or resume route recording."""
        self.status = "RECORDING"
        self.waypoints.clear()
        self.last_x = None
        self.last_y = None
        self.last_yaw = None
        self.last_time = 0.0

    def stop_recording(self):
        """Stop route recording."""
        if self.status == "RECORDING":
            self.status = "SAVED" if len(self.waypoints) > 0 else "OFF"

    def clear(self):
        """Clear recorded waypoints from memory."""
        self.waypoints.clear()
        self.status = "OFF"
        self.last_x = None
        self.last_y = None
        self.last_yaw = None
        self.last_time = 0.0

    def record_sample(
        self,
        x: float,
        y: float,
        yaw: float,
        speed: float,
        progress: float,
        sim_time: float,
        force: bool = False,
    ) -> bool:
        """
        Record a waypoint if compaction criteria are met.
        
        Returns:
            True if a new waypoint was added, False otherwise.
        """
        if self.status != "RECORDING" and not force:
            return False

        # First waypoint is always recorded
        if not self.waypoints:
            self._add_waypoint(x, y, yaw, speed, progress, sim_time)
            return True

        # Check distance traveled since last waypoint
        dist_moved = math.hypot(x - self.last_x, y - self.last_y)

        # Check heading difference (handling 360 wrap-around)
        yaw_diff = abs((yaw - self.last_yaw + 180.0) % 360.0 - 180.0)

        # Check time elapsed since last waypoint
        time_diff = sim_time - self.last_time

        # Record if any threshold is crossed, or forced (e.g. landing)
        if (
            force
            or dist_moved >= self.min_dist
            or yaw_diff >= self.min_yaw_diff
            or time_diff >= self.min_time_diff
        ):
            self._add_waypoint(x, y, yaw, speed, progress, sim_time)
            return True

        return False

    def _add_waypoint(
        self,
        x: float,
        y: float,
        yaw: float,
        speed: float,
        progress: float,
        sim_time: float,
    ):
        """Internal helper to append a waypoint."""
        waypoint = {
            "index": len(self.waypoints),
            "x": round(float(x), 2),
            "y": round(float(y), 2),
            "yaw": round(float(yaw), 2),
            "speed": round(float(speed), 2),
            "progress": round(float(progress), 2),
            "time": round(float(sim_time), 3),
        }
        self.waypoints.append(waypoint)
        self.last_x = float(x)
        self.last_y = float(y)
        self.last_yaw = float(yaw)
        self.last_time = float(sim_time)

    def save_to_json(self, filepath: Optional[str] = None) -> bool:
        """
        Save the mapped waypoints to a local JSON file.
        
        Returns:
            True if save succeeded, False otherwise.
        """
        target_path = filepath if filepath is not None else self.filename
        if not self.waypoints:
            return False

        total_distance = 0.0
        for i in range(len(self.waypoints) - 1):
            p1 = self.waypoints[i]
            p2 = self.waypoints[i + 1]
            total_distance += math.hypot(p2["x"] - p1["x"], p2["y"] - p1["y"])

        data = {
            "metadata": {
                "version": "1.0",
                "format": "autonomous_drone_path",
                "saved_timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
                "total_waypoints": len(self.waypoints),
                "total_distance_px": round(total_distance, 2),
                "flight_duration_sec": round(self.waypoints[-1]["time"], 3),
                "final_progress_pct": self.waypoints[-1]["progress"],
            },
            "waypoints": self.waypoints,
        }

        try:
            with open(target_path, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2)
            self.status = "SAVED"
            return True
        except Exception as e:
            print(f"[ERROR] Failed to save path to {target_path}: {e}")
            return False

    def load_from_json(self, filepath: Optional[str] = None) -> bool:
        """
        Load a previously mapped route from a JSON file.
        
        Returns:
            True if load succeeded, False otherwise.
        """
        target_path = filepath if filepath is not None else self.filename
        if not os.path.exists(target_path):
            return False

        try:
            with open(target_path, "r", encoding="utf-8") as f:
                data = json.load(f)

            waypoints = data.get("waypoints") if isinstance(data, dict) else None
            if self.validate_route(waypoints):
                self.waypoints = [dict(wp) for wp in waypoints]
                self.status = "SAVED"
                if self.waypoints:
                    self.last_x = float(self.waypoints[-1]["x"])
                    self.last_y = float(self.waypoints[-1]["y"])
                    self.last_yaw = float(self.waypoints[-1].get("yaw", 0.0))
                    self.last_time = float(self.waypoints[-1].get("time", 0.0))
                return True
            print(f"[ERROR] Invalid route data in {target_path}.")
            return False
        except Exception as e:
            print(f"[ERROR] Failed to load path from {target_path}: {e}")
            return False

    def get_waypoint_coordinates(self) -> List[Tuple[float, float]]:
        """Return a simple list of (x, y) tuples for visualization."""
        return [(wp["x"], wp["y"]) for wp in self.waypoints]

    @staticmethod
    def validate_route(waypoints: Any) -> bool:
        """Return whether waypoints contain a usable finite polyline."""
        if not isinstance(waypoints, list) or len(waypoints) < 2:
            return False
        points = []
        try:
            for waypoint in waypoints:
                if not isinstance(waypoint, dict):
                    return False
                x, y = float(waypoint["x"]), float(waypoint["y"])
                if not math.isfinite(x) or not math.isfinite(y):
                    return False
                points.append((x, y))
        except (KeyError, TypeError, ValueError):
            return False
        return any(math.hypot(bx - ax, by - ay) > 1e-6
                   for (ax, ay), (bx, by) in zip(points, points[1:]))


class RouteTracker:
    """Projects a drone onto a saved polyline and selects a forward lookahead."""

    def __init__(self, waypoints: List[Dict[str, Any]]):
        if not PathMapper.validate_route(waypoints):
            raise ValueError("Known path requires at least two valid route points.")

        self.waypoints = [dict(wp) for wp in waypoints]
        self.points = [(float(wp["x"]), float(wp["y"])) for wp in self.waypoints]
        self.lookahead_distance = float(KNOWN_PATH_LOOKAHEAD_DIST)
        self.search_ahead_segments = int(KNOWN_PATH_SEARCH_AHEAD_SEGMENTS)
        self.segment_lengths: List[float] = []
        self.cumulative_lengths: List[float] = [0.0]
        for a, b in zip(self.points, self.points[1:]):
            length = math.hypot(b[0] - a[0], b[1] - a[1])
            self.segment_lengths.append(length)
            self.cumulative_lengths.append(self.cumulative_lengths[-1] + length)
        self.vertex_turn_angles = [0.0] * len(self.points)
        for index in range(1, len(self.points) - 1):
            incoming = self.segment_lengths[index - 1]
            outgoing = self.segment_lengths[index]
            if incoming <= 1e-6 or outgoing <= 1e-6:
                continue
            a, b, c = self.points[index - 1:index + 2]
            incoming_heading = math.atan2(b[1] - a[1], b[0] - a[0])
            outgoing_heading = math.atan2(c[1] - b[1], c[0] - b[0])
            turn = (outgoing_heading - incoming_heading + math.pi) % (2.0 * math.pi) - math.pi
            self.vertex_turn_angles[index] = abs(math.degrees(turn))
        sharp_candidates = [
            index for index in range(1, len(self.points) - 1)
            if self.vertex_turn_angles[index] >= KNOWN_PATH_SHARP_TURN_THRESHOLD_DEG
        ]
        self.sharp_turn_distances = [
            self.cumulative_lengths[index]
            for index in sharp_candidates
            if sum(
                abs(self.cumulative_lengths[other] - self.cumulative_lengths[index])
                <= KNOWN_PATH_SHARP_TURN_CLUSTER_DIST / 2.0
                for other in sharp_candidates
            ) >= KNOWN_PATH_SHARP_TURN_MIN_VERTICES
        ]
        self.total_length = self.cumulative_lengths[-1]
        self.cursor_distance = 0.0
        self.cursor_segment: Optional[int] = None
        self.last_projection: Optional[Dict[str, Any]] = None

    @staticmethod
    def _project_to_segment(x, y, a, b):
        dx, dy = b[0] - a[0], b[1] - a[1]
        length_sq = dx * dx + dy * dy
        if length_sq <= 1e-12:
            t = 0.0
        else:
            t = max(0.0, min(1.0, ((x - a[0]) * dx + (y - a[1]) * dy) / length_sq))
        px, py = a[0] + t * dx, a[1] + t * dy
        return math.hypot(x - px, y - py), px, py, t

    def project(self, x: float, y: float, full_search: bool = False) -> Dict[str, Any]:
        """Find the nearest route projection, locally after the cursor advances."""
        if self.cursor_segment is None or full_search:
            first_segment, last_segment = 0, len(self.segment_lengths) - 1
        else:
            first_segment = max(0, self.cursor_segment - 1)
            last_segment = min(
                len(self.segment_lengths) - 1,
                self.cursor_segment + self.search_ahead_segments,
            )

        best = None
        for index in range(first_segment, last_segment + 1):
            distance, px, py, t = self._project_to_segment(
                float(x), float(y), self.points[index], self.points[index + 1]
            )
            along = self.cumulative_lengths[index] + t * self.segment_lengths[index]
            if best is None or distance < best["distance"]:
                best = {
                    "x": px,
                    "y": py,
                    "distance": distance,
                    "segment_index": index,
                    "segment_t": t,
                    "route_distance": along,
                }

        # The validation guarantees at least one non-zero route segment.
        self.cursor_distance = max(self.cursor_distance, best["route_distance"])
        self.cursor_segment = max(
            self.cursor_segment if self.cursor_segment is not None else 0,
            best["segment_index"],
        )
        best["cursor_distance"] = self.cursor_distance
        best["progress"] = 100.0 * self.cursor_distance / self.total_length
        self.last_projection = best
        return best

    def classify_turn_ahead(
        self,
        route_distance: Optional[float] = None,
        preview_distance: float = KNOWN_PATH_SPEED_LOOKAHEAD_DIST,
    ) -> str:
        """Classify the strongest waypoint turn within the forward preview."""
        start = self.cursor_distance if route_distance is None else max(0.0, float(route_distance))
        end = min(self.total_length, start + max(0.0, float(preview_distance)))
        if any(start <= distance <= end for distance in self.sharp_turn_distances):
            return "SHARP"
        max_turn = max(
            (
                self.vertex_turn_angles[index]
                for index in range(1, len(self.vertex_turn_angles) - 1)
                if start <= self.cumulative_lengths[index] <= end
            ),
            default=0.0,
        )
        if max_turn >= KNOWN_PATH_GENTLE_TURN_THRESHOLD_DEG:
            return "GENTLE"
        return "STRAIGHT"

    def target_speed_for_distance(
        self,
        route_distance: Optional[float] = None,
        preview_distance: float = KNOWN_PATH_SPEED_LOOKAHEAD_DIST,
    ) -> Tuple[float, str]:
        """Return geometry-derived target speed and its upcoming turn class."""
        turn_class = self.classify_turn_ahead(route_distance, preview_distance)
        target_speed = {
            "STRAIGHT": KNOWN_PATH_STRAIGHT_SPEED,
            "GENTLE": KNOWN_PATH_GENTLE_CURVE_SPEED,
            "SHARP": KNOWN_PATH_SHARP_TURN_SPEED,
        }[turn_class]
        return max(KNOWN_PATH_MIN_SPEED, min(KNOWN_PATH_MAX_SPEED, target_speed)), turn_class

    @staticmethod
    def adaptive_lookahead(speed: float, turn_class: str) -> float:
        """Scale look-ahead with speed and reduce it for upcoming curvature."""
        speed_fraction = max(
            0.0,
            min(1.0, (float(speed) - KNOWN_PATH_MIN_SPEED)
                / max(1e-6, KNOWN_PATH_MAX_SPEED - KNOWN_PATH_MIN_SPEED)),
        )
        distance = KNOWN_PATH_MIN_LOOKAHEAD_DIST + speed_fraction * (
            KNOWN_PATH_MAX_LOOKAHEAD_DIST - KNOWN_PATH_MIN_LOOKAHEAD_DIST
        )
        if turn_class == "GENTLE":
            distance *= KNOWN_PATH_GENTLE_LOOKAHEAD_SCALE
        elif turn_class == "SHARP":
            distance *= KNOWN_PATH_SHARP_LOOKAHEAD_SCALE
        return max(KNOWN_PATH_MIN_LOOKAHEAD_DIST, min(KNOWN_PATH_MAX_LOOKAHEAD_DIST, distance))

    def lookahead_target(self, lookahead_distance: Optional[float] = None) -> Dict[str, float]:
        """Interpolate a target ahead of the monotonic route cursor."""
        distance = self.lookahead_distance if lookahead_distance is None else max(0.0, float(lookahead_distance))
        target_distance = min(
            self.total_length,
            self.cursor_distance + distance,
        )
        segment = min(
            len(self.segment_lengths) - 1,
            max(0, bisect_right(self.cumulative_lengths, target_distance) - 1),
        )
        length = self.segment_lengths[segment]
        t = 0.0 if length <= 1e-12 else (
            (target_distance - self.cumulative_lengths[segment]) / length
        )
        a, b = self.points[segment], self.points[segment + 1]
        return {
            "x": a[0] + t * (b[0] - a[0]),
            "y": a[1] + t * (b[1] - a[1]),
            "route_distance": target_distance,
        }


class RouteSimilarityComparator:
    """Score live sensor line-center observations against saved route geometry."""

    def __init__(self, waypoints: List[Dict[str, Any]]):
        if not PathMapper.validate_route(waypoints):
            raise ValueError("Route comparison requires a valid saved route.")
        self.tracker = RouteTracker(waypoints)
        self.decision = "GATHERING EVIDENCE"
        self.similarity_score = 0.0
        self.confidence = 0.0
        self.comparison_count = 0
        self._scores: List[float] = []
        self._last_raw_point: Optional[Tuple[float, float]] = None
        self._last_sample_point: Optional[Tuple[float, float]] = None
        self._last_heading_point: Optional[Tuple[float, float]] = None
        self._travel_distance = 0.0
        self._anchored = False
        self._anchor_route_distance = 0.0
        self.covered_route_distance = 0.0

    @property
    def anchored(self) -> bool:
        return self._anchored

    def add_observation(self, point: Tuple[float, float]) -> Dict[str, Any]:
        """Add a detected line-center point and update score/confidence/decision."""
        x, y = float(point[0]), float(point[1])
        if not math.isfinite(x) or not math.isfinite(y):
            return self.snapshot()
        current = (x, y)
        if self._last_raw_point is not None:
            self._travel_distance += math.hypot(
                x - self._last_raw_point[0], y - self._last_raw_point[1]
            )
        self._last_raw_point = current

        if self.decision == "CHANGED ROUTE":
            return self.snapshot()

        if (
            self._last_sample_point is not None
            and math.hypot(x - self._last_sample_point[0], y - self._last_sample_point[1])
            < ROUTE_SIMILARITY_SAMPLE_DISTANCE
        ):
            self._update_decision()
            return self.snapshot()

        projection = self.tracker.project(x, y, full_search=not self._anchored)
        if not self._anchored:
            if projection["distance"] <= ROUTE_SIMILARITY_ANCHOR_DISTANCE:
                self._anchored = True
                self._anchor_route_distance = projection["route_distance"]
                self._last_sample_point = current
                self._last_heading_point = current
            elif self._travel_distance >= ROUTE_SIMILARITY_ANCHOR_TIMEOUT_DISTANCE:
                self.similarity_score = 0.0
                self.confidence = 1.0
                self.decision = "CHANGED ROUTE"
            return self.snapshot()

        segment = projection["segment_index"]
        a = self.tracker.points[segment]
        b = self.tracker.points[segment + 1]
        route_heading = math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))
        observed_heading = math.degrees(math.atan2(
            current[1] - self._last_heading_point[1],
            current[0] - self._last_heading_point[0],
        ))
        heading_error = abs((observed_heading - route_heading + 180.0) % 360.0 - 180.0)

        position_excess = max(
            0.0, projection["distance"] - ROUTE_SIMILARITY_POSITION_TOLERANCE
        )
        position_score = math.exp(-0.5 * (position_excess / ROUTE_SIMILARITY_POSITION_SCALE) ** 2)
        heading_score = math.exp(-0.5 * (heading_error / ROUTE_SIMILARITY_HEADING_SCALE_DEG) ** 2)
        self._scores.append(0.75 * position_score + 0.25 * heading_score)
        self._scores = self._scores[-20:]
        self.comparison_count += 1
        self.covered_route_distance = max(
            self.covered_route_distance,
            projection["cursor_distance"] - self._anchor_route_distance,
        )
        self._last_sample_point = current
        self._last_heading_point = current
        self.similarity_score = sum(self._scores) / len(self._scores)
        required_coverage = self.tracker.total_length * ROUTE_SIMILARITY_MIN_ROUTE_COVERAGE
        self.confidence = min(
            1.0,
            self.comparison_count / ROUTE_SIMILARITY_MIN_SAMPLES,
            self.covered_route_distance / required_coverage,
        )
        self._update_decision()
        return self.snapshot()

    def _update_decision(self):
        if self.decision == "CHANGED ROUTE":
            return
        required_coverage = self.tracker.total_length * ROUTE_SIMILARITY_MIN_ROUTE_COVERAGE
        if (
            self.comparison_count < ROUTE_SIMILARITY_MIN_SAMPLES
            or self.covered_route_distance < required_coverage
        ):
            self.decision = "GATHERING EVIDENCE"
        elif self.similarity_score >= ROUTE_SIMILARITY_ACCEPT_THRESHOLD:
            self.decision = "SIMILAR"
        elif self.similarity_score <= ROUTE_SIMILARITY_CHANGED_THRESHOLD:
            self.decision = "CHANGED ROUTE"
        elif self.similarity_score < ROUTE_SIMILARITY_UNCERTAIN_THRESHOLD:
            self.decision = "LOW SIMILARITY - UNCERTAIN"
        else:
            self.decision = "UNCERTAIN"

    def snapshot(self) -> Dict[str, Any]:
        return {
            "score": self.similarity_score,
            "confidence": self.confidence,
            "decision": self.decision,
            "samples": self.comparison_count,
            "anchored": self._anchored,
            "covered_route_distance": self.covered_route_distance,
            "required_route_distance": self.tracker.total_length * ROUTE_SIMILARITY_MIN_ROUTE_COVERAGE,
        }
