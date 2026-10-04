"""
main.py
=============================================================================
Main Entry Point for Autonomous Drone Line Follower Simulator (Stage 1).

Autonomous Line Following Prototype:
- Starts at the green Start Pad.
- Downward camera detects the amber line and calculates lateral error.
- Simple PD controller steers the drone smoothly along the course.
- Tracks and displays real-time telemetry: Speed, Lateral Error, Progress %, and Time.
- Automatically comes to a controlled stop/landing when reaching the red Landing Pad.

How to run:
    python main.py

Automated Test:
    python main.py --test
=============================================================================
"""

import sys
import os
import argparse
import tempfile
import math
import pygame

from config import (
    WINDOW_WIDTH,
    WINDOW_HEIGHT,
    FPS,
    SIM_TITLE,
    KNOWN_PATH_MIN_SPEED,
    KNOWN_PATH_MAX_SPEED,
    KNOWN_PATH_STRAIGHT_SPEED,
    KNOWN_PATH_GENTLE_CURVE_SPEED,
    KNOWN_PATH_SHARP_TURN_SPEED,
    AUTOPILOT_CRUISE_SPEED,
    KNOWN_PATH_ACCEL_LIMIT,
    KNOWN_PATH_DECEL_LIMIT,
    KNOWN_PATH_MIN_LOOKAHEAD_DIST,
    KNOWN_PATH_MAX_LOOKAHEAD_DIST,
    KNOWN_PATH_SPEED_LOOKAHEAD_DIST,
    AUTOPILOT_LINE_LOSS_RECOVERY_DURATION,
    AUTOPILOT_LINE_LOSS_RECOVERY_THROTTLE,
    DEFAULT_DISTRACTOR_LINE_COLOR,
)
from src.world import World
from src.drone import Drone
from src.sensors import LineSensor
from src.controller import DroneController, NavigationMode
from src.renderer import Renderer
from src.path_manager import PathMapper, RouteTracker, RouteSimilarityComparator


def load_known_route_snapshot(filename):
    """Load a fresh, validated route snapshot from the current saved file."""
    mapper = PathMapper(filename=filename)
    if not mapper.load_from_json():
        return None
    return tuple(dict(waypoint) for waypoint in mapper.waypoints)


def snapshot_from_waypoints(waypoints):
    """Copy a usable mapping result into the immutable navigation snapshot."""
    if not PathMapper.validate_route(waypoints):
        return None
    return tuple(dict(waypoint) for waypoint in waypoints)


def enter_known_path(controller, route_snapshot):
    """Apply the K-key transition using the latest validated route snapshot."""
    if (
        route_snapshot is None
        or controller.mode not in (NavigationMode.AUTOPILOT, NavigationMode.MANUAL)
    ):
        return False
    route = list(route_snapshot)
    controller.start_known_path(RouteTracker(route), RouteSimilarityComparator(route))
    return True


def enter_known_path_from_file(controller, filename, world=None, drone=None, sensor=None):
    """Refresh the route and enter Known Path, restarting from LANDED if needed."""
    snapshot = load_known_route_snapshot(filename)
    restarted = False
    if snapshot is not None and controller.mode == NavigationMode.LANDED:
        if world is None or drone is None or sensor is None:
            return False, snapshot, restarted
        drone.reset(x=world.start_pos[0], y=world.start_pos[1], yaw=world.initial_heading)
        controller.reset()
        # The current observation still belongs to the landing position. Refresh
        # it now so Known Path acquisition starts from the reset drone position.
        sensor.update(1.0 / FPS, drone.x, drone.y, drone.yaw, world)
        restarted = True
    return enter_known_path(controller, snapshot), snapshot, restarted


def start_clean_mapping_run(world, drone, controller, sensor, path_mapper):
    """Reset to course start and begin a fresh autonomous mapping run."""
    drone.reset(x=world.start_pos[0], y=world.start_pos[1], yaw=world.initial_heading)
    controller.reset()
    controller.mode = NavigationMode.PATH_MAPPING
    path_mapper.clear()
    path_mapper.start_recording()
    sensor.update(1.0 / FPS, drone.x, drone.y, drone.yaw, world)
    path_mapper.record_sample(
        x=drone.x,
        y=drone.y,
        yaw=drone.yaw,
        speed=drone.speed,
        progress=world.calculate_progress(drone.x, drone.y),
        sim_time=0.0,
    )


def finish_mapping_recording(path_mapper):
    """Stop and save a mapping run, returning its new usable route snapshot."""
    if path_mapper.status != "RECORDING":
        return False, None
    path_mapper.stop_recording()
    saved = path_mapper.save_to_json()
    return saved, snapshot_from_waypoints(path_mapper.waypoints) if saved else None

def parse_arguments():
    """Parse command line flags."""
    parser = argparse.ArgumentParser(description="Autonomous Drone Line Follower Simulator")
    parser.add_argument(
        "--test",
        action="store_true",
        help="Run headless simulation to verify autonomous line following from Start to Landing Pad.",
    )
    parser.add_argument(
        "--test-mapping",
        action="store_true",
        help="Run a headless path mapping, save, and load verification.",
    )
    parser.add_argument(
        "--test-known-path",
        action="store_true",
        help="Run a headless known-path tracking and fallback verification.",
    )
    parser.add_argument(
        "--test-route-similarity",
        action="store_true",
        help="Run matching, noisy-offset, and changed-route similarity checks.",
    )
    parser.add_argument(
        "--test-stage4-edge-cases",
        action="store_true",
        help="Run Stage 4 route replacement and evidence-gating edge cases.",
    )
    parser.add_argument(
        "--test-clean-mapping",
        action="store_true",
        help="Run a full-course clean mapping, save, and Known Path availability test.",
    )
    parser.add_argument(
        "--test-stage5",
        action="store_true",
        help="Run Known Path speed-profile, adaptive look-ahead, and smoothing tests.",
    )
    parser.add_argument(
        "--test-known-path-key",
        action="store_true",
        help="Verify K-key activation reloads and enters a valid saved route.",
    )
    parser.add_argument(
        "--test-line-loss",
        action="store_true",
        help="Verify bounded line-loss recovery and unchanged normal following.",
    )
    parser.add_argument(
        "--test-color-sensing",
        action="store_true",
        help="Verify target-color sensing rejects crossing and nearby distractor lines.",
    )
    return parser.parse_args()


def run_line_loss_test() -> bool:
    """Check temporary recovery, reacquisition, bounded hold, and normal controls."""
    print("\n[TEST] Running bounded AUTOPILOT line-loss recovery checks...")
    dt = 1.0 / FPS
    world = World()

    # A short line loss retains the existing reduced-throttle, last-error search.
    controller = DroneController()
    drone = Drone(world.start_pos[0], world.start_pos[1], world.initial_heading)
    sensor = LineSensor()
    sensor.line_detected = False
    sensor.last_known_error = 9.0
    throttle = steering = 0.0
    for _ in range(10):
        throttle, steering = controller.update(dt, drone.get_state(), sensor, world)
    if not (
        throttle == AUTOPILOT_LINE_LOSS_RECOVERY_THROTTLE
        and steering > 0.0
        and controller._line_loss_elapsed < AUTOPILOT_LINE_LOSS_RECOVERY_DURATION
    ):
        print("[TEST FAILURE] Temporary line loss did not continue controlled recovery.")
        return False

    # A detected line immediately clears the timer and restores normal controls.
    sensor.line_detected = True
    sensor.lateral_error = 4.0
    throttle, _ = controller.update(dt, drone.get_state(), sensor, world)
    if throttle != 0.72 or controller._line_loss_elapsed != 0.0:
        print("[TEST FAILURE] Line reacquisition did not restore normal following/reset timer.")
        return False

    # Verify the existing detected-line throttle bands remain exactly intact.
    for error, expected in ((0.0, 0.72), (20.0, 0.52), (30.0, 0.38)):
        check = DroneController()
        detected = LineSensor()
        detected.line_detected = True
        detected.lateral_error = error
        result, _ = check.update(dt, drone.get_state(), detected, world)
        if result != expected:
            print("[TEST FAILURE] Normal detected-line throttle behavior changed.")
            return False

    # Sustained loss gets zero motion commands and the drone settles to a stop.
    controller = DroneController()
    drone = Drone(world.start_pos[0], world.start_pos[1], world.initial_heading)
    sensor = LineSensor()
    sensor.line_detected = False
    sensor.last_known_error = -6.0
    stopped = False
    for _ in range(int((AUTOPILOT_LINE_LOSS_RECOVERY_DURATION + 2.5) / dt)):
        throttle, steering = controller.update(dt, drone.get_state(), sensor, world)
        drone.set_inputs(throttle, steering)
        drone.update(dt)
        if controller._line_loss_elapsed > AUTOPILOT_LINE_LOSS_RECOVERY_DURATION:
            stopped = throttle == 0.0 and steering == 0.0
    if not stopped or drone.speed > 0.1:
        print("[TEST FAILURE] Prolonged line loss did not stop/hold the drone.")
        return False

    print(
        "[TEST SUCCESS] temporary recovery, reacquisition, configured timeout, "
        "and unchanged normal throttle bands verified.\n"
    )
    return True


def run_color_sensing_test() -> bool:
    """Verify target-color sensing through overlap and full-course flights."""
    print("\n[TEST] Running target-color and distractor-line checks...")
    target_points = [(120.0, 350.0), (880.0, 350.0)]
    distractors = [
        {
            "waypoints": [(500.0, 200.0), (500.0, 500.0)],
            "color": DEFAULT_DISTRACTOR_LINE_COLOR,
        },
        {
            "waypoints": [(280.0, 375.0), (460.0, 375.0)],
            "color": (220, 70, 70),
        },
    ]
    world_with_distractors = World(target_points, distractor_lines=distractors)
    sensor = LineSensor(lookahead=0.0, span=0.0)

    # A nearby distractor is visible in the probe bar but must not shift the
    # target-line centroid. This remains a deterministic color-label sample.
    sensor.update(1.0 / FPS, 318.0, 350.0, 0.0, world_with_distractors)
    if not sensor.line_detected or sensor.observed_color != world_with_distractors.target_line_color:
        print("[TEST FAILURE] Sensor did not recognize the configured target color.")
        return False
    centered_sensor = LineSensor(lookahead=0.0, span=100.0)
    centered_sensor.update(1.0 / FPS, 350.0, 350.0, 0.0, world_with_distractors)
    if (
        not centered_sensor.line_detected
        or centered_sensor.observed_color != world_with_distractors.target_line_color
        or abs(centered_sensor.lateral_error) > 2.1
    ):
        print("[TEST FAILURE] Nearby distractor contaminated the target-line observation.")
        return False

    # At the crossing, the later-drawn distractor occludes the target color.
    # Sensor must reject that sample rather than switch to the distractor.
    crossing_color = world_with_distractors.sample_line_color(500.0, 350.0)
    sensor.update(1.0 / FPS, 500.0, 350.0, 0.0, world_with_distractors)
    if crossing_color == world_with_distractors.target_line_color or sensor.line_detected:
        print("[TEST FAILURE] Sensor accepted a distractor color at the crossing.")
        return False

    def fly_to_landing(world):
        drone = Drone(world.start_pos[0], world.start_pos[1], world.initial_heading)
        line_sensor = LineSensor()
        controller = DroneController()
        dt = 1.0 / FPS
        line_sensor.update(dt, drone.x, drone.y, drone.yaw, world)
        line_losses = 0
        max_loss_elapsed = 0.0
        observations_on_target = True
        completed = False
        for _ in range(6000):
            if not line_sensor.line_detected:
                line_losses += 1
            throttle, steering = controller.update(dt, drone.get_state(), line_sensor, world)
            max_loss_elapsed = max(max_loss_elapsed, controller._line_loss_elapsed)
            if line_sensor.line_detected:
                observations_on_target = observations_on_target and (
                    line_sensor.observed_color == world.target_line_color
                    and world.query_point_on_path(*line_sensor.observed_path_point)[0]
                )
            drone.set_inputs(throttle, steering)
            drone.update(dt)
            line_sensor.update(dt, drone.x, drone.y, drone.yaw, world)
            if controller.course_completed:
                completed = True
                break
        return completed, drone, line_losses, max_loss_elapsed, observations_on_target

    plain_world = World(target_points)
    plain_result = fly_to_landing(plain_world)
    if not plain_result[0] or plain_result[2] != 0 or not plain_result[4]:
        print("[TEST FAILURE] Target line did not remain normally trackable without distractors.")
        return False

    distractor_result = fly_to_landing(world_with_distractors)
    if not (
        distractor_result[0]
        and distractor_result[2] > 0
        and distractor_result[3] <= AUTOPILOT_LINE_LOSS_RECOVERY_DURATION
        and distractor_result[4]
        and world_with_distractors.is_at_landing_pad(
            distractor_result[1].x, distractor_result[1].y
        )
    ):
        print("[TEST FAILURE] Drone did not recover from the crossing and finish on the target line.")
        return False

    print(
        "[TEST SUCCESS] target detection, nearby/crossing distractor rejection, "
        "normal no-distractor flight, and recovery through brief occlusion verified.\n"
    )
    return True


def run_headless_test() -> bool:
    """
    Automated verification test: runs full autonomous line-following flight
    headlessly until landing or timeout.
    """
    print("\n[TEST] Running headless automated line-following verification...")
    os.environ["SDL_VIDEODRIVER"] = "dummy"
    pygame.init()

    world = World()
    drone = Drone(
        start_x=world.start_pos[0],
        start_y=world.start_pos[1],
        start_yaw=world.initial_heading,
    )
    sensor = LineSensor()
    controller = DroneController()

    dt = 1.0 / FPS
    max_frames = 2000  # Up to ~33 simulated seconds
    elapsed_time = 0.0
    completed = False

    print(f"[TEST] Starting position: ({drone.x:.1f}, {drone.y:.1f}), Heading: {drone.yaw:.1f}°")
    print(f"[TEST] Target Landing Pad: ({world.landing_pos[0]:.1f}, {world.landing_pos[1]:.1f})")

    # Initial sensor check
    sensor.update(dt, drone.x, drone.y, drone.yaw, world)
    assert sensor.line_detected, "Line must be detected at the Start Pad"
    print(f"[TEST PASS] Sensor line detection initialized. Lateral error: {sensor.lateral_error:.2f} px")

    # Autonomous flight loop
    for frame in range(max_frames):
        if not completed:
            elapsed_time += dt

        throttle, steering = controller.update(dt, drone.get_state(), sensor, world, keys=None)
        drone.set_inputs(throttle, steering)
        drone.update(dt)
        sensor.update(dt, drone.x, drone.y, drone.yaw, world)

        progress = world.calculate_progress(drone.x, drone.y)

        # Log milestones every 200 frames (~3.3 seconds)
        if frame % 200 == 0:
            print(f"  Frame {frame:4d} | Pos: ({drone.x:5.1f}, {drone.y:5.1f}) | Speed: {drone.speed:4.1f} px/s | Error: {sensor.lateral_error:+5.1f} px | Progress: {progress:5.1f}%")

        if controller.mode == NavigationMode.LANDED or controller.course_completed:
            completed = True
            print(f"\n[TEST PASS] Drone successfully reached and landed at destination!")
            print(f"  Final Progress: {progress:.1f}%")
            print(f"  Total Flight Time: {elapsed_time:.2f} seconds")
            print(f"  Final Position: ({drone.x:.1f}, {drone.y:.1f}) (Distance to Landing Pad: {((drone.x - world.landing_pos[0])**2 + (drone.y - world.landing_pos[1])**2)**0.5:.1f} px)")
            break

    pygame.quit()

    if not completed:
        print("[TEST FAILURE] Drone timed out before reaching the landing pad.")
        return False

    assert progress > 90.0, "Progress should be near 100% at completion"
    print("[TEST SUCCESS] Stage 1 Autonomous Line-Following Prototype Verified!\n")
    return True


def run_path_mapping_test() -> bool:
    """Verify recording a route, saving it, and loading it headlessly."""
    print("\n[TEST] Running headless path mapping save/load verification...")
    with tempfile.TemporaryDirectory() as temp_dir:
        filepath = os.path.join(temp_dir, "saved_path.json")
        mapper = PathMapper(filename=filepath)
        mapper.start_recording()
        mapper.record_sample(120.0, 560.0, 0.0, 0.0, 0.0, 0.0)
        mapper.record_sample(140.0, 560.0, 0.0, 45.0, 5.0, 0.5)
        mapper.record_sample(160.0, 560.0, 0.0, 45.0, 10.0, 1.0, force=True)
        mapper.stop_recording()

        if not mapper.save_to_json():
            print("[TEST FAILURE] Could not save mapped route.")
            return False

        loaded_mapper = PathMapper(filename=filepath)
        if not loaded_mapper.load_from_json():
            print("[TEST FAILURE] Could not load saved mapped route.")
            return False

        if loaded_mapper.waypoints != mapper.waypoints or len(loaded_mapper.waypoints) != 3:
            print("[TEST FAILURE] Loaded route did not match the recorded waypoints.")
            return False

    print("[TEST SUCCESS] Path mapping, save, and load verified.\n")
    return True


def run_clean_mapping_test() -> bool:
    """Verify a clean mapping starts at zero progress and saves the full course."""
    print("\n[TEST] Running clean full-course path mapping verification...")
    with tempfile.TemporaryDirectory() as temp_dir:
        filepath = os.path.join(temp_dir, "saved_path.json")
        world = World()
        drone = Drone(world.start_pos[0] + 90.0, world.start_pos[1] + 70.0, 180.0)
        sensor = LineSensor()
        controller = DroneController()
        mapper = PathMapper(filename=filepath)

        start_clean_mapping_run(world, drone, controller, sensor, mapper)
        first = mapper.waypoints[0]
        if (
            controller.mode != NavigationMode.PATH_MAPPING
            or (drone.x, drone.y, drone.yaw)
            != (world.start_pos[0], world.start_pos[1], world.initial_heading)
            or first["progress"] > 0.1
            or abs(first["x"] - world.start_pos[0]) > 1.0
            or abs(first["y"] - world.start_pos[1]) > 1.0
        ):
            print("[TEST FAILURE] Clean mapping did not begin at the course start.")
            return False

        dt = 1.0 / FPS
        completed = False
        for _ in range(3000):
            throttle, steering = controller.update(dt, drone.get_state(), sensor, world)
            drone.set_inputs(throttle, steering)
            drone.update(dt)
            sensor.update(dt, drone.x, drone.y, drone.yaw, world)
            progress = world.calculate_progress(drone.x, drone.y)
            mapper.record_sample(
                drone.x, drone.y, drone.yaw, drone.speed, progress,
                (_ + 1) * dt, force=controller.course_completed,
            )
            if controller.course_completed:
                completed = True
                break

        if not completed or not world.is_at_landing_pad(drone.x, drone.y):
            print("[TEST FAILURE] Clean mapping did not reach the landing area.")
            return False
        saved, route_snapshot = finish_mapping_recording(mapper)
        if not saved or route_snapshot is None:
            print("[TEST FAILURE] Completed mapping did not save a usable route.")
            return False
        final = mapper.waypoints[-1]
        if final["progress"] < 90.0 or not world.is_at_landing_pad(final["x"], final["y"]):
            print("[TEST FAILURE] Forced final waypoint was not saved at completion.")
            return False

        controller.reset()
        if not enter_known_path(controller, route_snapshot):
            print("[TEST FAILURE] Newly mapped route was not immediately available to Known Path.")
            return False
        if controller.route_tracker.waypoints != mapper.waypoints:
            print("[TEST FAILURE] Known Path did not receive the newly mapped route.")
            return False

    print("[TEST SUCCESS] Clean start, full-course recording, final waypoint, save, and Known Path availability verified.\n")
    return True


def run_known_path_test() -> bool:
    """Verify acquisition, route projection/lookahead, sensor correction, and fallback."""
    print("\n[TEST] Running headless Known Path verification...")
    route = [
        {"x": 100.0, "y": 560.0},
        {"x": 200.0, "y": 560.0},
        {"x": 300.0, "y": 560.0},
    ]
    if not PathMapper.validate_route(route) or PathMapper.validate_route([{"x": 1, "y": 2}]):
        print("[TEST FAILURE] Route validation did not accept/reject expected inputs.")
        return False

    world = World()
    sensor = LineSensor()
    sensor.line_detected = True
    sensor.lateral_error = 0.0
    controller = DroneController()
    controller.start_known_path(RouteTracker(route), None)
    controller.similarity_decision = "SIMILAR"

    # The route begins 100 px away, so the controller should acquire it using
    # line following before handing off to the saved route.
    state = {"x": 0.0, "y": 560.0, "yaw": 0.0}
    controller.update(1.0 / FPS, state, sensor, world)
    if controller.known_path_state != "ACQUIRING ROUTE":
        print("[TEST FAILURE] Controller did not remain in route acquisition.")
        return False

    state["x"] = 40.0  # 60 px from the first route point: within join radius
    controller.update(1.0 / FPS, state, sensor, world)
    if controller.known_path_state != "TRACKING ROUTE" or controller.lookahead_target is None:
        print("[TEST FAILURE] Controller did not acquire the route or select a target.")
        return False

    state["x"] = 145.0
    controller.update(1.0 / FPS, state, sensor, world)
    if controller.route_tracker.cursor_distance < 40.0 or controller.lookahead_target["x"] <= state["x"]:
        print("[TEST FAILURE] Route cursor/lookahead did not advance ahead of the drone.")
        return False

    sensor.lateral_error = 20.0
    controller.update(1.0 / FPS, state, sensor, world)
    if not (0.0 < controller.sensor_correction <= 0.20):
        print("[TEST FAILURE] Sensor correction was not applied within its bound.")
        return False

    state["y"] = 700.0
    sensor.line_detected = False
    throttle, steering = controller.update(1.0 / FPS, state, sensor, world)
    if controller.known_path_state != "ROUTE LOST - HOLD" or throttle != 0.0 or steering != 0.0:
        print("[TEST FAILURE] Lost route and line did not trigger neutral hold.")
        return False

    sensor.line_detected = True
    throttle, _ = controller.update(1.0 / FPS, state, sensor, world)
    if controller.known_path_state != "LINE FOLLOWING FALLBACK" or throttle <= 0.0:
        print("[TEST FAILURE] Visible line did not activate line-following fallback.")
        return False

    # Prefer the current saved route so a partial route also exercises
    # acquisition from the Start Pad. Fall back to course geometry if absent.
    saved_route = PathMapper()
    if saved_route.load_from_json():
        flight_route = saved_route.waypoints
    else:
        flight_route = [{"x": x, "y": y} for x, y in world.waypoints]
    flight_tracker = RouteTracker(flight_route)
    flight_controller = DroneController()
    flight_controller.start_known_path(
        flight_tracker, RouteSimilarityComparator(flight_route)
    )
    drone = Drone(world.start_pos[0], world.start_pos[1], world.initial_heading)
    flight_sensor = LineSensor()
    dt = 1.0 / FPS
    flight_sensor.update(dt, drone.x, drone.y, drone.yaw, world)
    landed = False
    for _ in range(3000):
        throttle, steering = flight_controller.update(
            dt, drone.get_state(), flight_sensor, world
        )
        drone.set_inputs(throttle, steering)
        drone.update(dt)
        flight_sensor.update(dt, drone.x, drone.y, drone.yaw, world)
        if flight_controller.course_completed:
            landed = True
            break
    if (
        not landed
        or not world.is_at_landing_pad(drone.x, drone.y)
        or flight_controller.similarity_decision != "SIMILAR"
        or not flight_controller._route_guidance_active
    ):
        print("[TEST FAILURE] Similarity-gated Known Path did not complete at the landing pad.")
        return False

    print("[TEST SUCCESS] Acquisition, route tracking, bounded correction, fallback, and landing verified.\n")
    return True


def run_stage5_test() -> bool:
    """Verify geometry speed classes, adaptive look-ahead, and slew limits."""
    print("\n[TEST] Running Stage 5 speed profile and adaptive look-ahead checks...")
    straight = RouteTracker([
        {"x": 0.0, "y": 0.0}, {"x": 200.0, "y": 0.0}, {"x": 400.0, "y": 0.0}
    ])
    gentle = RouteTracker([
        {"x": 0.0, "y": 0.0}, {"x": 100.0, "y": 0.0},
        {"x": 200.0, "y": 40.0}, {"x": 300.0, "y": 80.0},
    ])
    sharp = RouteTracker([
        {"x": 0.0, "y": 0.0},
        {"x": 30.0, "y": 0.0},
        {"x": 59.71, "y": 4.18},
        {"x": 88.40, "y": 12.45},
        {"x": 115.91, "y": 24.63},
    ])
    straight_speed, straight_class = straight.target_speed_for_distance(0.0)
    gentle_speed, gentle_class = gentle.target_speed_for_distance(0.0)
    sharp_speed, sharp_class = sharp.target_speed_for_distance(0.0)
    if not (
        straight_class == "STRAIGHT"
        and gentle_class == "GENTLE"
        and sharp_class == "SHARP"
        and (straight_speed, gentle_speed, sharp_speed) == (75.0, 62.0, 48.0)
        and (KNOWN_PATH_STRAIGHT_SPEED, KNOWN_PATH_GENTLE_CURVE_SPEED, KNOWN_PATH_SHARP_TURN_SPEED)
        == (75.0, 62.0, 48.0)
        and KNOWN_PATH_MAX_SPEED <= AUTOPILOT_CRUISE_SPEED
        and KNOWN_PATH_SPEED_LOOKAHEAD_DIST == 110.0
        and KNOWN_PATH_MAX_SPEED >= straight_speed > gentle_speed > sharp_speed >= KNOWN_PATH_MIN_SPEED
    ):
        print("[TEST FAILURE] Geometry did not produce the configured 75/62/48 px/s profile within bounds.")
        return False

    saved_route = PathMapper()
    if not saved_route.load_from_json():
        print("[TEST FAILURE] Could not load saved_path.json for course geometry classification.")
        return False
    saved_tracker = RouteTracker(saved_route.waypoints)
    sampled_classes = [
        saved_tracker.classify_turn_ahead(float(distance))
        for distance in range(0, int(saved_tracker.total_length) + 1, 10)
    ]
    route_class_counts = {
        turn: sampled_classes.count(turn)
        for turn in ("STRAIGHT", "GENTLE", "SHARP")
    }
    route_class_proportions = {
        turn: route_class_counts[turn] / len(sampled_classes)
        for turn in ("STRAIGHT", "GENTLE", "SHARP")
    }
    if not (
        route_class_proportions["STRAIGHT"] >= 0.10
        and route_class_proportions["GENTLE"] >= 0.20
        and 0.10 <= route_class_proportions["SHARP"] <= 0.50
    ):
        print(f"[TEST FAILURE] Saved route did not yield a sensible turn-class mix: {route_class_proportions}.")
        return False

    isolated_kink = RouteTracker([
        {"x": 0.0, "y": 0.0},
        {"x": 100.0, "y": 0.0},
        {"x": 200.0, "y": 17.63},
        {"x": 300.0, "y": 17.63},
    ])
    if isolated_kink.classify_turn_ahead(0.0) == "SHARP":
        print("[TEST FAILURE] Isolated waypoint noise was classified as a sharp turn.")
        return False

    fast_lookahead = straight.adaptive_lookahead(straight_speed, "STRAIGHT")
    slow_lookahead = straight.adaptive_lookahead(sharp_speed, "STRAIGHT")
    gentle_lookahead = straight.adaptive_lookahead(straight_speed, "GENTLE")
    sharp_lookahead = straight.adaptive_lookahead(straight_speed, "SHARP")
    lookaheads = [
        straight.adaptive_lookahead(speed, turn)
        for speed in (0.0, KNOWN_PATH_MIN_SPEED, KNOWN_PATH_MAX_SPEED, 300.0)
        for turn in ("STRAIGHT", "GENTLE", "SHARP")
    ]
    if not (
        fast_lookahead > slow_lookahead
        and fast_lookahead > gentle_lookahead > sharp_lookahead
        and all(KNOWN_PATH_MIN_LOOKAHEAD_DIST <= d <= KNOWN_PATH_MAX_LOOKAHEAD_DIST for d in lookaheads)
    ):
        print("[TEST FAILURE] Adaptive look-ahead did not respond to speed/curvature within bounds.")
        return False

    controller = DroneController()
    previous = controller._ramp_known_path_speed(straight_speed, 0.1)
    if previous > KNOWN_PATH_ACCEL_LIMIT * 0.1 + 1e-9:
        print("[TEST FAILURE] Known Path acceleration exceeded its configured limit.")
        return False
    for _ in range(30):
        previous = controller._ramp_known_path_speed(straight_speed, 0.1)
    if not (KNOWN_PATH_MIN_SPEED <= previous <= KNOWN_PATH_MAX_SPEED):
        print("[TEST FAILURE] Smoothed speed escaped its configured bounds.")
        return False
    reduced = controller._ramp_known_path_speed(sharp_speed, 0.1)
    if previous - reduced > KNOWN_PATH_DECEL_LIMIT * 0.1 + 1e-9:
        print("[TEST FAILURE] Known Path deceleration exceeded its configured limit.")
        return False

    print(
        "[TEST SUCCESS] "
        f"targets={straight_speed:.0f}/{gentle_speed:.0f}/{sharp_speed:.0f} px/s, "
        f"look-ahead={fast_lookahead:.1f}/{sharp_lookahead:.1f} px; "
        "saved-route mix="
        f"{route_class_counts['STRAIGHT']}/{route_class_counts['GENTLE']}/{route_class_counts['SHARP']} "
        f"of {len(sampled_classes)} samples "
        f"({route_class_proportions['STRAIGHT']:.0%}/{route_class_proportions['GENTLE']:.0%}/"
        f"{route_class_proportions['SHARP']:.0%}); noise rejection and slew limits verified.\n"
    )
    return True


def run_known_path_key_test() -> bool:
    """Verify K restart refreshes sensor state and completes route acquisition."""
    print("\n[TEST] Running Known Path restart-after-landing regression check...")
    with tempfile.TemporaryDirectory() as temp_dir:
        filepath = os.path.join(temp_dir, "saved_path.json")
        mapper = PathMapper(filename=filepath)
        world = World()
        drone = Drone(world.start_pos[0], world.start_pos[1], world.initial_heading)
        sensor = LineSensor()
        controller = DroneController()
        mapper.start_recording()
        dt = 1.0 / FPS
        elapsed = 0.0
        sensor.update(dt, drone.x, drone.y, drone.yaw, world)
        mapper.record_sample(
            drone.x, drone.y, drone.yaw, drone.speed,
            world.calculate_progress(drone.x, drone.y), elapsed,
        )

        # Complete a real line-following flight and use its recorded route as
        # the saved Known Path reference for the restart.
        for _ in range(6000):
            throttle, steering = controller.update(dt, drone.get_state(), sensor, world)
            drone.set_inputs(throttle, steering)
            drone.update(dt)
            elapsed += dt
            sensor.update(dt, drone.x, drone.y, drone.yaw, world)
            progress = world.calculate_progress(drone.x, drone.y)
            mapper.record_sample(
                drone.x, drone.y, drone.yaw, drone.speed, progress, elapsed,
                force=controller.course_completed,
            )
            if controller.course_completed:
                break
        mapper.stop_recording()
        if not controller.course_completed or not mapper.save_to_json():
            print("[TEST FAILURE] Could not complete and save the pre-K route.")
            return False

        old_observation = sensor.observed_path_point
        if old_observation is None:
            print("[TEST FAILURE] Landing did not leave a sensor observation to invalidate.")
            return False
        activated, refreshed_snapshot, restarted = enter_known_path_from_file(
            controller, filepath, world, drone, sensor
        )
        if (
            not activated
            or not restarted
            or controller.mode != NavigationMode.KNOWN_PATH
            or controller.route_tracker is None
            or controller.route_tracker.waypoints != list(refreshed_snapshot or ())
            or (drone.x, drone.y, drone.yaw)
            != (world.start_pos[0], world.start_pos[1], world.initial_heading)
            or sensor.observed_path_point is None
            or sensor.observed_path_point == old_observation
            or math.hypot(
                sensor.observed_path_point[0] - world.start_pos[0],
                sensor.observed_path_point[1] - world.start_pos[1],
            ) > 60.0
        ):
            print("[TEST FAILURE] K restart did not refresh the sensor at course start.")
            return False

        # The first controller update must anchor at the new start observation,
        # then continue collecting evidence until route guidance is enabled.
        controller.update(dt, drone.get_state(), sensor, world)
        comparator = controller.similarity_comparator
        if (
            comparator is None
            or not comparator.anchored
            or comparator._anchor_route_distance > 60.0
        ):
            print("[TEST FAILURE] Similarity acquisition anchored to the stale landing observation.")
            return False
        sensor.update(dt, drone.x, drone.y, drone.yaw, world)

        saw_similar = False
        saw_guidance = False
        for _ in range(6000):
            throttle, steering = controller.update(dt, drone.get_state(), sensor, world)
            saw_similar = saw_similar or controller.similarity_decision == "SIMILAR"
            saw_guidance = saw_guidance or controller._route_guidance_active
            drone.set_inputs(throttle, steering)
            drone.update(dt)
            sensor.update(dt, drone.x, drone.y, drone.yaw, world)
            if controller.course_completed:
                break
        if not saw_similar or not saw_guidance:
            print("[TEST FAILURE] Restarted Known Path did not pass similarity acquisition into route guidance.")
            return False

    print("[TEST SUCCESS] K restart refreshed the sensor, similarity passed, and route guidance activated.\n")
    return True


def run_route_similarity_test() -> bool:
    """Verify online geometry scoring for matching, noisy, and changed paths."""
    print("\n[TEST] Running route similarity verification...")
    reference = [
        {"x": 0.0, "y": 0.0},
        {"x": 200.0, "y": 0.0},
        {"x": 400.0, "y": 0.0},
        {"x": 400.0, "y": 200.0},
        {"x": 400.0, "y": 400.0},
    ]

    def feed_polyline(comparator, points):
        for start, end in zip(points, points[1:]):
            distance = ((end[0] - start[0]) ** 2 + (end[1] - start[1]) ** 2) ** 0.5
            steps = max(1, int(distance / 10.0))
            for step in range(steps):
                t = step / steps
                comparator.add_observation((
                    start[0] + t * (end[0] - start[0]),
                    start[1] + t * (end[1] - start[1]),
                ))
        comparator.add_observation(points[-1])
        return comparator.snapshot()

    route_points = [(wp["x"], wp["y"]) for wp in reference]
    matching = RouteSimilarityComparator(reference)
    matching_result = feed_polyline(matching, route_points)
    if matching_result["decision"] != "SIMILAR":
        print(f"[TEST FAILURE] Matching route scored {matching_result}.")
        return False

    shifted_noisy = RouteSimilarityComparator(reference)
    noisy_points = []
    for index, (x, y) in enumerate(route_points):
        noise = (0.0, 1.5, -1.5, 0.5)[index % 4]
        noisy_points.append((x + 2.0, y + 7.0 + noise))
    noisy_result = feed_polyline(shifted_noisy, noisy_points)
    if noisy_result["decision"] != "SIMILAR":
        print(f"[TEST FAILURE] Shifted/noisy route scored {noisy_result}.")
        return False

    uncertain_reference = [{"x": 0.0, "y": 0.0}, {"x": 500.0, "y": 0.0}]
    uncertain = RouteSimilarityComparator(uncertain_reference)
    uncertain_result = feed_polyline(uncertain, [(0.0, 40.0), (320.0, 40.0)])
    if uncertain_result["decision"] not in ("UNCERTAIN", "LOW SIMILARITY - UNCERTAIN"):
        print(f"[TEST FAILURE] Borderline route was not classified uncertain: {uncertain_result}.")
        return False

    changed = RouteSimilarityComparator(reference)
    changed_result = feed_polyline(changed, [(0.0, 100.0), (320.0, 100.0)])
    if changed_result["decision"] != "CHANGED ROUTE":
        print(f"[TEST FAILURE] Clearly different route scored {changed_result}.")
        return False

    # A route sharing its opening straight must remain in acquisition until
    # evidence reaches and evaluates the later divergence.
    shared_prefix = RouteSimilarityComparator(reference)
    for x in range(0, 201, 10):
        prefix_result = shared_prefix.add_observation((float(x), 0.0))
    prefix_controller = DroneController()
    prefix_sensor = LineSensor()
    prefix_sensor.line_detected = True
    prefix_sensor.observed_path_point = (200.0, 0.0)
    prefix_controller.start_known_path(RouteTracker(reference), shared_prefix)
    prefix_controller.update(
        1.0 / FPS, {"x": 200.0, "y": 0.0, "yaw": 0.0}, prefix_sensor, World()
    )
    if (
        prefix_result["decision"] == "SIMILAR"
        or prefix_controller._route_guidance_active
        or prefix_controller.known_path_state != "ACQUIRING"
    ):
        print("[TEST FAILURE] Shared opening section was accepted before divergence evidence.")
        return False
    divergence_result = feed_polyline(
        shared_prefix, [(200.0, 0.0), (200.0, -400.0)]
    )
    if divergence_result["decision"] == "SIMILAR":
        print("[TEST FAILURE] Divergent route was accepted as matching.")
        return False

    # Verify decisions actually gate controller handoff.
    gate = RouteSimilarityComparator(reference)
    gate_result = feed_polyline(gate, route_points)
    if gate_result["decision"] != "SIMILAR":
        print(f"[TEST FAILURE] Full matching route did not satisfy coverage gate: {gate_result}.")
        return False
    sensor = LineSensor()
    sensor.line_detected = True
    sensor.observed_path_point = (80.0, 0.0)
    controller = DroneController()
    controller.start_known_path(RouteTracker(reference), gate)
    controller.update(1.0 / FPS, {"x": 80.0, "y": 0.0, "yaw": 0.0}, sensor, World())
    if controller.known_path_state != "TRACKING ROUTE":
        print("[TEST FAILURE] High similarity did not enable route guidance.")
        return False

    pending = RouteSimilarityComparator(reference)
    pending.add_observation((0.0, 0.0))  # Anchor, but not enough scored samples.
    pending_sensor = LineSensor()
    pending_sensor.line_detected = True
    pending_sensor.observed_path_point = (10.0, 0.0)
    pending_controller = DroneController()
    pending_controller.start_known_path(RouteTracker(reference), pending)
    pending_controller.update(
        1.0 / FPS, {"x": 10.0, "y": 0.0, "yaw": 0.0}, pending_sensor, World()
    )
    if pending_controller._route_guidance_active or pending_controller.known_path_state != "ACQUIRING":
        print("[TEST FAILURE] Insufficient evidence enabled route guidance.")
        return False

    changed_sensor = LineSensor()
    changed_sensor.line_detected = True
    changed_sensor.observed_path_point = (320.0, 100.0)
    changed_controller = DroneController()
    changed_controller.start_known_path(RouteTracker(reference), changed)
    changed_controller.update(
        1.0 / FPS, {"x": 320.0, "y": 100.0, "yaw": 0.0}, changed_sensor, World()
    )
    if changed_controller.mode != NavigationMode.AUTOPILOT:
        print("[TEST FAILURE] Changed route did not disable Known Path guidance.")
        return False

    print(
        "[TEST SUCCESS] "
        f"matching={matching_result['score']:.2f}, "
        f"shifted/noisy={noisy_result['score']:.2f}, "
        f"borderline={uncertain_result['score']:.2f}, "
        f"changed={changed_result['score']:.2f}.\n"
    )
    return True


def run_stage4_edge_case_test() -> bool:
    """Verify route replacement activation and incomplete evidence stays gated."""
    print("\n[TEST] Running Stage 4 integration edge cases...")
    with tempfile.TemporaryDirectory() as temp_dir:
        filepath = os.path.join(temp_dir, "saved_path.json")
        mapper = PathMapper(filename=filepath)
        mapper.start_recording()
        mapper.record_sample(10.0, 10.0, 0.0, 0.0, 0.0, 0.0)
        mapper.record_sample(30.0, 10.0, 0.0, 0.0, 1.0, 1.0, force=True)
        mapper.stop_recording()
        if not mapper.save_to_json():
            print("[TEST FAILURE] Could not save replacement route.")
            return False

        replacement_snapshot = snapshot_from_waypoints(mapper.waypoints)
        if replacement_snapshot is None:
            print("[TEST FAILURE] Newly mapped replacement route was not usable.")
            return False
        controller = DroneController()
        if not enter_known_path(controller, replacement_snapshot):
            print("[TEST FAILURE] K transition rejected the newly saved route.")
            return False
        if controller.route_tracker.waypoints != list(replacement_snapshot):
            print("[TEST FAILURE] K activated a route other than the replacement route.")
            return False
        controller.reset()
        if (
            controller.route_tracker is not None
            or controller.similarity_comparator is not None
            or controller.mode != NavigationMode.AUTOPILOT
        ):
            print("[TEST FAILURE] Reset did not clear Known Path runtime state.")
            return False
        reset_snapshot = load_known_route_snapshot(filepath)
        if not enter_known_path(controller, reset_snapshot):
            print("[TEST FAILURE] Reset could not reactivate the current saved route.")
            return False
        if controller.route_tracker.waypoints != list(replacement_snapshot):
            print("[TEST FAILURE] Reset reactivated a stale route snapshot.")
            return False

    short_route = [
        {"x": 0.0, "y": 0.0},
        {"x": 300.0, "y": 0.0},
        {"x": 300.0, "y": 300.0},
    ]
    comparator = RouteSimilarityComparator(short_route)
    for x in range(0, 81, 8):
        result = comparator.add_observation((float(x), 0.0))
    controller = DroneController()
    sensor = LineSensor()
    sensor.line_detected = True
    sensor.observed_path_point = (80.0, 0.0)
    if not enter_known_path(controller, tuple(short_route)):
        print("[TEST FAILURE] Could not enter Known Path for evidence-gate check.")
        return False
    controller.similarity_comparator = comparator
    controller.update(1.0 / FPS, {"x": 80.0, "y": 0.0, "yaw": 0.0}, sensor, World())
    if (
        result["decision"] == "SIMILAR"
        or controller._route_guidance_active
        or controller.known_path_state != "ACQUIRING"
    ):
        print("[TEST FAILURE] Incomplete route observations activated Known Path.")
        return False

    print("[TEST SUCCESS] Replacement route is used immediately; short evidence remains gated.\n")
    return True


def main():
    args = parse_arguments()

    if args.test:
        success = run_headless_test()
        sys.exit(0 if success else 1)
    if args.test_mapping:
        success = run_path_mapping_test()
        sys.exit(0 if success else 1)
    if args.test_clean_mapping:
        success = run_clean_mapping_test()
        sys.exit(0 if success else 1)
    if args.test_known_path:
        success = run_known_path_test()
        sys.exit(0 if success else 1)
    if args.test_stage5:
        success = run_stage5_test()
        sys.exit(0 if success else 1)
    if args.test_known_path_key:
        success = run_known_path_key_test()
        sys.exit(0 if success else 1)
    if args.test_line_loss:
        success = run_line_loss_test()
        sys.exit(0 if success else 1)
    if args.test_color_sensing:
        success = run_color_sensing_test()
        sys.exit(0 if success else 1)
    if args.test_route_similarity:
        success = run_route_similarity_test()
        sys.exit(0 if success else 1)
    if args.test_stage4_edge_cases:
        success = run_stage4_edge_case_test()
        sys.exit(0 if success else 1)

    # 1. Initialize Pygame Graphics Window
    pygame.init()
    pygame.display.set_caption(SIM_TITLE)
    screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
    clock = pygame.time.Clock()

    # 2. Instantiate Simulator Subsystems
    world = World()
    drone = Drone(
        start_x=world.start_pos[0],
        start_y=world.start_pos[1],
        start_yaw=world.initial_heading,
    )
    sensor = LineSensor()
    controller = DroneController()
    renderer = Renderer(screen)
    path_mapper = PathMapper()
    if path_mapper.load_from_json():
        print(f"[INFO] Loaded {len(path_mapper.waypoints)} mapped waypoints from {path_mapper.filename}.")
        # Mapping can clear PathMapper.waypoints, so keep navigation's route
        # independent from the mutable recording buffer.
        known_route_snapshot = tuple(dict(wp) for wp in path_mapper.waypoints)
    else:
        known_route_snapshot = None

    def finish_mapping():
        nonlocal known_route_snapshot
        if path_mapper.status == "RECORDING":
            saved, route_snapshot = finish_mapping_recording(path_mapper)
            if saved:
                print(f"[INFO] Saved {len(path_mapper.waypoints)} mapped waypoints to {path_mapper.filename}.")
                known_route_snapshot = route_snapshot

    def refresh_known_route_snapshot():
        nonlocal known_route_snapshot
        known_route_snapshot = load_known_route_snapshot(path_mapper.filename)

    elapsed_time = 0.0
    running = True

    print("\n" + "=" * 65)
    print(" Autonomous Drone Line Follower Simulator (Stage 1)")
    print(" Active Mode: AUTOPILOT (Line Following)")
    print(" Controls:")
    print("   [R] Reset Flight Run")
    print("   [M] Toggle Autopilot / Manual Flight")
    print("   [P] Start Clean Full-Course Mapping Run")
    print("   [K] Toggle Known Path (when a valid saved route is loaded)")
    print("   [ESC] Exit")
    print("=" * 65 + "\n")

    # 3. Main Simulation Loop
    while running:
        dt = clock.tick(FPS) / 1000.0  # Seconds since last frame
        if dt > 0.1:
            dt = 0.1  # Prevent time jumps if window is moved

        # Handle Events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_r:
                    finish_mapping()
                    refresh_known_route_snapshot()
                    # Reset flight
                    drone.reset(
                        x=world.start_pos[0],
                        y=world.start_pos[1],
                        yaw=world.initial_heading,
                    )
                    controller.reset()
                    renderer.reset_trail()
                    elapsed_time = 0.0
                    print("[INFO] Flight reset to Start Pad.")
                elif event.key == pygame.K_p:
                    start_clean_mapping_run(world, drone, controller, sensor, path_mapper)
                    renderer.reset_trail()
                    elapsed_time = 0.0
                    known_route_snapshot = None
                    print("[MODE] Started clean PATH_MAPPING run from the course start.")
                elif event.key == pygame.K_m:
                    if controller.mode == NavigationMode.KNOWN_PATH:
                        controller.clear_known_path(clear_similarity=False)
                        controller.mode = NavigationMode.MANUAL
                        controller.known_path_state = "MANUAL OVERRIDE"
                        print("[MODE] Switched to MANUAL pilot mode.")
                    elif controller.mode == NavigationMode.MANUAL:
                        controller.mode = NavigationMode.PATH_MAPPING
                        path_mapper.start_recording()
                        print("[MODE] Switched to PATH MAPPING.")
                    elif controller.mode == NavigationMode.PATH_MAPPING:
                        controller.mode = NavigationMode.AUTOPILOT
                        finish_mapping()
                        print("[MODE] Switched to AUTOPILOT (Line Following).")
                    else:
                        controller.mode = NavigationMode.MANUAL
                        print("[MODE] Switched to MANUAL pilot mode.")
                elif event.key == pygame.K_k:
                    if controller.mode == NavigationMode.KNOWN_PATH:
                        controller.mode = NavigationMode.AUTOPILOT
                        controller.clear_known_path()
                        print("[MODE] Switched to AUTOPILOT (Line Following).")
                    else:
                        activated, known_route_snapshot, restarted = enter_known_path_from_file(
                            controller, path_mapper.filename, world, drone, sensor
                        )
                        if activated:
                            if restarted:
                                renderer.reset_trail()
                                elapsed_time = 0.0
                            print("[MODE] Switched to KNOWN_PATH (acquiring saved route).")
                        else:
                            print("[INFO] Known Path unavailable: load a valid saved route and leave mapping mode first.")

        # Update Timer (only when flight is in progress)
        if controller.mode in (
            NavigationMode.AUTOPILOT,
            NavigationMode.PATH_MAPPING,
            NavigationMode.KNOWN_PATH,
        ) and not controller.course_completed:
            elapsed_time += dt

        # Read keys (for manual override)
        keys = pygame.key.get_pressed()

        # Controller Update
        throttle, steering = controller.update(dt, drone.get_state(), sensor, world, keys)

        # Drone Kinematics Update
        drone.set_inputs(throttle, steering)
        drone.update(dt)


        # Sensor Sample Update
        sensor.update(dt, drone.x, drone.y, drone.yaw, world)

        # Progress Calculation
        progress = world.calculate_progress(drone.x, drone.y)
                # Record path while mapping
        if path_mapper.status == "RECORDING":
            path_mapper.record_sample(
                x=drone.x,
                y=drone.y,
                yaw=drone.yaw,
                speed=drone.speed,
                progress=progress,
                sim_time=elapsed_time,
                force=controller.course_completed,
            )
            if controller.course_completed:
                finish_mapping()

        # Render Display Frame
        rendered_route = (
            controller.route_tracker.waypoints
            if controller.route_tracker is not None
            else path_mapper.waypoints
        )
        renderer.render(
            world=world,
            drone=drone,
            sensor=sensor,
            mode_str=controller.mode.value,
            progress=progress,
            elapsed_time=elapsed_time,
            is_completed=controller.course_completed,
            fps=clock.get_fps(),
            mapped_waypoints=rendered_route,
            mapping_status=path_mapper.status,
            known_path_state=controller.known_path_state,
            route_progress=controller.route_progress,
            route_distance=controller.route_distance,
            lookahead_target=controller.lookahead_target,
            similarity_score=controller.similarity_score,
            similarity_confidence=controller.similarity_confidence,
            similarity_decision=controller.similarity_decision,
        )

    finish_mapping()
    pygame.quit()
    print("[INFO] Simulation closed cleanly.")


if __name__ == "__main__":
    main()
