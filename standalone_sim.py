"""
standalone_sim.py
=============================================================================
University Drone Competition - Autonomous Line Follower Prototype (Stage 1)
Single-File Self-Contained Simulation.

Everything needed to run the 2D simulation is contained in this single file:
- World track geometry and progress calculation
- Drone kinematics and rotor animation
- Downward camera line sensor with optical probe array
- Autopilot PD line follower and adaptive speed modulation
- Pygame visualizer and Telemetry Heads-Up Display (HUD)

Run with:
    python standalone_sim.py
    python standalone_sim.py --test
=============================================================================
"""

import sys
import os
import math
import argparse
import pygame
from typing import List, Tuple, Dict, Any

# ===========================================================================
# 1. CONFIGURATION CONSTANTS
# ===========================================================================
WINDOW_WIDTH = 1000
WINDOW_HEIGHT = 700
FPS = 60
SIM_TITLE = "Autonomous Drone Simulator - Stage 1 (Single-File Prototype)"

# Colors (RGB)
COLOR_BG = (24, 26, 32)
COLOR_GRID = (36, 40, 50)
COLOR_TEXT = (240, 240, 240)
COLOR_TEXT_DIM = (160, 165, 175)
COLOR_HUD_BG = (15, 17, 22, 215)
COLOR_PATH = (245, 176, 65)
COLOR_PATH_BORDER = (195, 135, 30)
COLOR_START_PAD = (46, 204, 113)
COLOR_LAND_PAD = (231, 76, 60)
COLOR_DRONE_BODY = (41, 128, 185)
COLOR_DRONE_NOSE = (230, 126, 34)
COLOR_ROTOR_ARM = (127, 140, 141)
COLOR_ROTOR_BLADE = (200, 214, 229)
COLOR_TRAIL = (52, 152, 219)
COLOR_SENSOR_LINE = (52, 152, 219)
COLOR_PROBE_ACTIVE = (46, 204, 113)
COLOR_PROBE_IDLE = (120, 120, 120)
COLOR_ERROR_INDICATOR = (241, 196, 15)

# Physics & Sensor Constants
DRONE_RADIUS = 20.0
MAX_SPEED = 180.0
CRUISE_SPEED = 90.0
MAX_TURN_RATE = 180.0
PATH_LINE_WIDTH = 18
SENSOR_LOOKAHEAD = 32.0
SENSOR_SPAN = 100.0
NUM_PROBES = 25

# Controller Gains
KP = 0.035  # Proportional steering gain
KD = 0.20   # Derivative damping gain

# Course Waypoints
COURSE_WAYPOINTS = [
    (120, 560), (260, 560), (380, 510), (480, 410),
    (560, 300), (660, 240), (760, 240), (840, 310),
    (880, 400), (880, 480), (830, 540), (740, 560),
    (640, 560), (520, 560)
]


# ===========================================================================
# 2. WORLD & TRACK GEOMETRY
# ===========================================================================
class World:
    def __init__(self, waypoints=COURSE_WAYPOINTS):
        self.waypoints = [(float(x), float(y)) for x, y in waypoints]
        self.line_width = float(PATH_LINE_WIDTH)
        self.start_pos = self.waypoints[0]
        self.landing_pos = self.waypoints[-1]
        self.pad_radius = 30.0

        # Initial heading facing the first segment
        dx = self.waypoints[1][0] - self.waypoints[0][0]
        dy = self.waypoints[1][1] - self.waypoints[0][1]
        self.initial_heading = math.degrees(math.atan2(dy, dx)) % 360.0

        # Precompute segment lengths for progress tracking
        self.segment_lengths = []
        self.cumulative_lengths = [0.0]
        total = 0.0
        for i in range(len(self.waypoints) - 1):
            seg_len = math.hypot(
                self.waypoints[i + 1][0] - self.waypoints[i][0],
                self.waypoints[i + 1][1] - self.waypoints[i][1],
            )
            self.segment_lengths.append(seg_len)
            total += seg_len
            self.cumulative_lengths.append(total)
        self.total_path_length = total

    def point_to_segment(self, px, py, ax, ay, bx, by):
        seg_sq = (bx - ax) ** 2 + (by - ay) ** 2
        if seg_sq == 0.0:
            return math.hypot(px - ax, py - ay), (ax, ay), 0.0
        t = max(0.0, min(1.0, ((px - ax) * (bx - ax) + (py - ay) * (by - ay)) / seg_sq))
        cx = ax + t * (bx - ax)
        cy = ay + t * (by - ay)
        return math.hypot(px - cx, py - cy), (cx, cy), t

    def query_point(self, px, py):
        half_w = self.line_width / 2.0
        min_d = float("inf")
        for i in range(len(self.waypoints) - 1):
            ax, ay = self.waypoints[i]
            bx, by = self.waypoints[i + 1]
            dist, _, _ = self.point_to_segment(px, py, ax, ay, bx, by)
            if dist < min_d:
                min_d = dist
        return min_d <= half_w

    def calculate_progress(self, drone_x, drone_y):
        if self.is_at_landing_pad(drone_x, drone_y):
            return 100.0
        min_d = float("inf")
        best_i = 0
        best_t = 0.0
        for i in range(len(self.waypoints) - 1):
            ax, ay = self.waypoints[i]
            bx, by = self.waypoints[i + 1]
            dist, _, t = self.point_to_segment(drone_x, drone_y, ax, ay, bx, by)
            if dist < min_d:
                min_d = dist
                best_i = i
                best_t = t
        dist_along = self.cumulative_lengths[best_i] + best_t * self.segment_lengths[best_i]
        return max(0.0, min(100.0, (dist_along / self.total_path_length) * 100.0))

    def is_at_landing_pad(self, x, y):
        return math.hypot(x - self.landing_pos[0], y - self.landing_pos[1]) <= self.pad_radius


# ===========================================================================
# 3. DRONE KINEMATICS
# ===========================================================================
class Drone:
    def __init__(self, x, y, yaw):
        self.start_x = float(x)
        self.start_y = float(y)
        self.start_yaw = float(yaw)
        self.reset()
        self.radius = DRONE_RADIUS

    def reset(self):
        self.x = self.start_x
        self.y = self.start_y
        self.yaw = self.start_yaw
        self.speed = 0.0
        self.rotor_angle = 0.0
        self.throttle_cmd = 0.0
        self.steering_cmd = 0.0

    def set_inputs(self, throttle, steering):
        self.throttle_cmd = max(-1.0, min(1.0, float(throttle)))
        self.steering_cmd = max(-1.0, min(1.0, float(steering)))

    def update(self, dt):
        # Update orientation
        yaw_rate = self.steering_cmd * MAX_TURN_RATE
        self.yaw = (self.yaw + yaw_rate * dt) % 360.0

        # Update speed
        target_speed = self.throttle_cmd * CRUISE_SPEED
        self.speed += (target_speed - self.speed) * min(1.0, 6.0 * dt)
        self.speed = max(-MAX_SPEED * 0.3, min(MAX_SPEED, self.speed))

        # Update position
        rad = math.radians(self.yaw)
        self.x += self.speed * math.cos(rad) * dt
        self.y += self.speed * math.sin(rad) * dt

        # Propeller rotation animation
        self.rotor_angle = (self.rotor_angle + (900.0 + abs(self.speed) * 6.0) * dt) % 360.0


# ===========================================================================
# 4. DOWNWARD CAMERA LINE SENSOR
# ===========================================================================
class LineSensor:
    def __init__(self):
        self.lookahead = SENSOR_LOOKAHEAD
        self.span = SENSOR_SPAN
        self.num_probes = NUM_PROBES
        self.line_detected = False
        self.lateral_error = 0.0
        self.last_known_error = 0.0
        self.active_probe_count = 0
        self.probes_data = []

    def update(self, drone_x, drone_y, drone_yaw, world: World):
        rad = math.radians(drone_yaw)
        fwd_x, fwd_y = math.cos(rad), math.sin(rad)
        right_x, right_y = -fwd_y, fwd_x

        cx = drone_x + fwd_x * self.lookahead
        cy = drone_y + fwd_y * self.lookahead

        half_span = self.span / 2.0
        step = self.span / (self.num_probes - 1)

        self.probes_data.clear()
        detected_offsets = []

        for i in range(self.num_probes):
            offset = -half_span + i * step
            px = cx + right_x * offset
            py = cy + right_y * offset
            detected = world.query_point(px, py)
            if detected:
                detected_offsets.append(offset)
            self.probes_data.append({"x": px, "y": py, "detected": detected})

        self.active_probe_count = len(detected_offsets)
        if self.active_probe_count > 0:
            self.line_detected = True
            self.lateral_error = sum(detected_offsets) / self.active_probe_count
            self.last_known_error = self.lateral_error
        else:
            self.line_detected = False
            self.lateral_error = self.last_known_error


# ===========================================================================
# 5. AUTOPILOT CONTROLLER (PD ALGORITHM)
# ===========================================================================
class Controller:
    def __init__(self):
        self.mode = "AUTOPILOT"
        self.kp = KP
        self.kd = KD
        self.prev_error = 0.0
        self.course_completed = False

    def reset(self):
        self.mode = "AUTOPILOT"
        self.prev_error = 0.0
        self.course_completed = False

    def compute(self, drone: Drone, sensor: LineSensor, world: World, keys=None):
        if self.mode == "MANUAL":
            throttle = 0.0
            steering = 0.0
            if keys:
                if keys[pygame.K_SPACE]: return 0.0, 0.0
                if keys[pygame.K_UP] or keys[pygame.K_w]: throttle += 1.0
                if keys[pygame.K_DOWN] or keys[pygame.K_s]: throttle -= 0.6
                if keys[pygame.K_LEFT] or keys[pygame.K_a]: steering -= 1.0
                if keys[pygame.K_RIGHT] or keys[pygame.K_d]: steering += 1.0
            return throttle, steering

        progress = world.calculate_progress(drone.x, drone.y)

        # Check Landing Pad
        if world.is_at_landing_pad(drone.x, drone.y) and progress > 85.0:
            self.mode = "LANDED"
            self.course_completed = True
            return 0.0, 0.0

        if self.mode == "LANDED":
            return 0.0, 0.0

        # Autonomous PD Line Follower
        if sensor.line_detected:
            error = sensor.lateral_error
            delta_e = error - self.prev_error
            self.prev_error = error

            # PD steering correction
            raw_steer = self.kp * error + self.kd * delta_e
            steering = max(-1.0, min(1.0, raw_steer))

            # Adaptive speed modulation
            abs_err = abs(error)
            if abs_err < 12.0:
                throttle = 0.72   # Full cruise on straight path
            elif abs_err < 25.0:
                throttle = 0.52   # Gentle speed on curves
            else:
                throttle = 0.38   # Cautious speed on sharp turns
        else:
            # Recovery
            steering = 0.6 if sensor.last_known_error > 0 else -0.6
            throttle = 0.30

        return throttle, steering


# ===========================================================================
# 6. PYGAME RENDERER & TELEMETRY HUD
# ===========================================================================
class Renderer:
    def __init__(self, screen):
        self.screen = screen
        self.trail = []
        pygame.font.init()
        try:
            self.font_main = pygame.font.SysFont("Consolas", 14)
            self.font_bold = pygame.font.SysFont("Consolas", 15, bold=True)
            self.font_title = pygame.font.SysFont("Arial", 16, bold=True)
            self.font_pad = pygame.font.SysFont("Arial", 11, bold=True)
            self.font_banner = pygame.font.SysFont("Arial", 22, bold=True)
        except Exception:
            self.font_main = pygame.font.Font(None, 18)
            self.font_bold = pygame.font.Font(None, 20)
            self.font_title = pygame.font.Font(None, 22)
            self.font_pad = pygame.font.Font(None, 14)
            self.font_banner = pygame.font.Font(None, 26)

    def reset_trail(self):
        self.trail.clear()

    def record_trail(self, x, y):
        if not self.trail or math.hypot(x - self.trail[-1][0], y - self.trail[-1][1]) > 3.0:
            self.trail.append((x, y))
            if len(self.trail) > 1200:
                self.trail.pop(0)

    def draw_world(self, world: World):
        # Path outer border
        for i in range(len(world.waypoints) - 1):
            p1, p2 = world.waypoints[i], world.waypoints[i + 1]
            pygame.draw.line(self.screen, COLOR_PATH_BORDER, p1, p2, int(world.line_width) + 4)
            pygame.draw.circle(self.screen, COLOR_PATH_BORDER, (int(p1[0]), int(p1[1])), int(world.line_width // 2) + 2)
            pygame.draw.circle(self.screen, COLOR_PATH_BORDER, (int(p2[0]), int(p2[1])), int(world.line_width // 2) + 2)

        # Main amber path
        for i in range(len(world.waypoints) - 1):
            p1, p2 = world.waypoints[i], world.waypoints[i + 1]
            pygame.draw.line(self.screen, COLOR_PATH, p1, p2, int(world.line_width))
            pygame.draw.circle(self.screen, COLOR_PATH, (int(p1[0]), int(p1[1])), int(world.line_width // 2))
            pygame.draw.circle(self.screen, COLOR_PATH, (int(p2[0]), int(p2[1])), int(world.line_width // 2))

        # Start Pad
        sx, sy = int(world.start_pos[0]), int(world.start_pos[1])
        pygame.draw.circle(self.screen, COLOR_START_PAD, (sx, sy), int(world.pad_radius))
        pygame.draw.circle(self.screen, (255, 255, 255), (sx, sy), int(world.pad_radius * 0.7), 2)
        txt = self.font_pad.render("START", True, (255, 255, 255))
        self.screen.blit(txt, (sx - txt.get_width() // 2, sy - txt.get_height() // 2))

        # Landing Pad
        lx, ly = int(world.landing_pos[0]), int(world.landing_pos[1])
        pygame.draw.circle(self.screen, COLOR_LAND_PAD, (lx, ly), int(world.pad_radius))
        pygame.draw.circle(self.screen, (255, 255, 255), (lx, ly), int(world.pad_radius * 0.7), 2)
        txt = self.font_pad.render("LAND", True, (255, 255, 255))
        self.screen.blit(txt, (lx - txt.get_width() // 2, ly - txt.get_height() // 2))

    def draw_sensor(self, drone: Drone, sensor: LineSensor):
        rad = math.radians(drone.yaw)
        fwd_x, fwd_y = math.cos(rad), math.sin(rad)
        cx = drone.x + fwd_x * sensor.lookahead
        cy = drone.y + fwd_y * sensor.lookahead
        pygame.draw.line(self.screen, COLOR_SENSOR_LINE, (drone.x, drone.y), (cx, cy), 1)

        if sensor.probes_data:
            p_left = sensor.probes_data[0]
            p_right = sensor.probes_data[-1]
            pygame.draw.line(self.screen, (70, 90, 110), (p_left["x"], p_left["y"]), (p_right["x"], p_right["y"]), 2)

        for p in sensor.probes_data:
            px, py = int(p["x"]), int(p["y"])
            if p["detected"]:
                pygame.draw.circle(self.screen, COLOR_PROBE_ACTIVE, (px, py), 4)
            else:
                pygame.draw.circle(self.screen, COLOR_PROBE_IDLE, (px, py), 2)

        if sensor.line_detected:
            right_x, right_y = -fwd_y, fwd_x
            ex = int(cx + right_x * sensor.lateral_error)
            ey = int(cy + right_y * sensor.lateral_error)
            pygame.draw.circle(self.screen, COLOR_ERROR_INDICATOR, (ex, ey), 6)

    def draw_drone(self, drone: Drone):
        dx, dy = drone.x, drone.y
        rad = math.radians(drone.yaw)
        arm_len = drone.radius * 1.3
        for r_ang in [math.pi / 4, 3 * math.pi / 4, 5 * math.pi / 4, 7 * math.pi / 4]:
            rx = dx + arm_len * math.cos(rad + r_ang)
            ry = dy + arm_len * math.sin(rad + r_ang)
            pygame.draw.line(self.screen, COLOR_ROTOR_ARM, (dx, dy), (rx, ry), 3)
            blade_r = 9.0
            spin = math.radians(drone.rotor_angle)
            bx1 = rx + blade_r * math.cos(spin)
            by1 = ry + blade_r * math.sin(spin)
            bx2 = rx - blade_r * math.cos(spin)
            by2 = ry - blade_r * math.sin(spin)
            pygame.draw.line(self.screen, COLOR_ROTOR_BLADE, (bx1, by1), (bx2, by2), 2)

        pygame.draw.circle(self.screen, COLOR_DRONE_BODY, (int(dx), int(dy)), int(drone.radius))
        pygame.draw.circle(self.screen, (255, 255, 255), (int(dx), int(dy)), 4)

        # Heading triangle
        nose_len = drone.radius * 1.4
        nose_tip = (dx + nose_len * math.cos(rad), dy + nose_len * math.sin(rad))
        p_left = (dx + drone.radius * 0.7 * math.cos(rad + 2.5), dy + drone.radius * 0.7 * math.sin(rad + 2.5))
        p_right = (dx + drone.radius * 0.7 * math.cos(rad - 2.5), dy + drone.radius * 0.7 * math.sin(rad - 2.5))
        pygame.draw.polygon(self.screen, COLOR_DRONE_NOSE, [nose_tip, p_left, p_right])

    def draw_hud(self, drone: Drone, sensor: LineSensor, mode_str: str, progress: float, elapsed: float, fps: float):
        hud_w, hud_h = 370, 240
        hud_surface = pygame.Surface((hud_w, hud_h), pygame.SRCALPHA)
        hud_surface.fill(COLOR_HUD_BG)
        pygame.draw.rect(hud_surface, (70, 80, 100), (0, 0, hud_w, hud_h), 1)

        title = self.font_title.render("DRONE TELEMETRY (STAGE 1)", True, (245, 176, 65))
        hud_surface.blit(title, (14, 10))

        err_text = "N/A"
        err_col = (231, 76, 60)
        if sensor.line_detected:
            if abs(sensor.lateral_error) < 2.0:
                err_text = f"{sensor.lateral_error:+.1f} px [CENTERED]"
                err_col = (46, 204, 113)
            elif sensor.lateral_error < 0:
                err_text = f"{sensor.lateral_error:+.1f} px [LINE LEFT]"
                err_col = (52, 152, 219)
            else:
                err_text = f"{sensor.lateral_error:+.1f} px [LINE RIGHT]"
                err_col = (230, 126, 34)

        time_str = f"{int(elapsed // 60):02d}:{elapsed % 60:04.1f} s"
        lines = [
            ("Flight Mode:", mode_str, (255, 255, 255)),
            ("Speed:", f"{drone.speed:.1f} px/s", (100, 220, 255)),
            ("Lateral Error:", err_text, err_col),
            ("Progress:", f"{progress:5.1f} %", (46, 204, 113) if progress >= 99.0 else COLOR_TEXT),
            ("Elapsed Time:", time_str, (241, 196, 15)),
            ("Simulation Rate:", f"{fps:.0f} FPS", COLOR_TEXT_DIM),
        ]

        y_offset = 36
        for label, val, color in lines:
            lbl = self.font_main.render(label, True, COLOR_TEXT_DIM)
            v = self.font_bold.render(val, True, color)
            hud_surface.blit(lbl, (14, y_offset))
            hud_surface.blit(v, (140, y_offset))
            y_offset += 20

        # Visual progress bar
        bx, by, bw, bh = 14, y_offset + 6, hud_w - 28, 10
        pygame.draw.rect(hud_surface, (40, 45, 55), (bx, by, bw, bh), border_radius=3)
        fill_w = int((progress / 100.0) * bw)
        if fill_w > 0:
            pygame.draw.rect(hud_surface, (46, 204, 113), (bx, by, fill_w, bh), border_radius=3)
        pygame.draw.rect(hud_surface, (90, 100, 120), (bx, by, bw, bh), 1, border_radius=3)

        self.screen.blit(hud_surface, (15, 15))

    def render(self, world, drone, sensor, controller, progress, elapsed, fps):
        self.record_trail(drone.x, drone.y)
        self.screen.fill(COLOR_BG)

        # Floor grid
        for x in range(0, WINDOW_WIDTH, 50):
            pygame.draw.line(self.screen, COLOR_GRID, (x, 0), (x, WINDOW_HEIGHT), 1)
        for y in range(0, WINDOW_HEIGHT, 50):
            pygame.draw.line(self.screen, COLOR_GRID, (0, y), (WINDOW_WIDTH, y), 1)

        self.draw_world(world)

        # Flight trail
        if len(self.trail) > 1:
            pygame.draw.lines(self.screen, COLOR_TRAIL, False, self.trail, 2)

        self.draw_sensor(drone, sensor)
        self.draw_drone(drone)
        self.draw_hud(drone, sensor, controller.mode, progress, elapsed, fps)

        # Landing celebration banner
        if controller.course_completed:
            bw, bh = 500, 75
            bx, by = (WINDOW_WIDTH - bw) // 2, 25
            bsurf = pygame.Surface((bw, bh), pygame.SRCALPHA)
            bsurf.fill((20, 30, 25, 230))
            pygame.draw.rect(bsurf, (46, 204, 113), (0, 0, bw, bh), 2, border_radius=8)
            t1 = self.font_banner.render("COURSE COMPLETED! LANDED AT TARGET", True, (46, 204, 113))
            t2 = self.font_main.render(f"Total Flight Time: {elapsed:.2f} s | Press [R] to Reset", True, (240, 240, 240))
            bsurf.blit(t1, (bw // 2 - t1.get_width() // 2, 12))
            bsurf.blit(t2, (bw // 2 - t2.get_width() // 2, 44))
            self.screen.blit(bsurf, (bx, by))

        pygame.display.flip()


# ===========================================================================
# 7. MAIN EXECUTION & AUTOMATED TEST
# ===========================================================================
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--test", action="store_true", help="Run automated headless verification")
    args = parser.parse_args()

    if args.test:
        os.environ["SDL_VIDEODRIVER"] = "dummy"
        pygame.init()
        world = World()
        drone = Drone(world.start_pos[0], world.start_pos[1], world.initial_heading)
        sensor = LineSensor()
        controller = Controller()

        dt = 1.0 / 60.0
        elapsed = 0.0
        for frame in range(2000):
            sensor.update(drone.x, drone.y, drone.yaw, world)
            throttle, steer = controller.compute(drone, sensor, world)
            drone.set_inputs(throttle, steer)
            drone.update(dt)
            if not controller.course_completed:
                elapsed += dt
            if controller.course_completed:
                print(f"[TEST SUCCESS] Standalone simulator landed in {elapsed:.2f} seconds!")
                print(f"Final Position: ({drone.x:.1f}, {drone.y:.1f}), Progress: {world.calculate_progress(drone.x, drone.y):.1f}%")
                pygame.quit()
                sys.exit(0)
        print("[TEST FAILURE] Drone timed out before landing.")
        pygame.quit()
        sys.exit(1)

    pygame.init()
    pygame.display.set_caption(SIM_TITLE)
    screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
    clock = pygame.time.Clock()

    world = World()
    drone = Drone(world.start_pos[0], world.start_pos[1], world.initial_heading)
    sensor = LineSensor()
    controller = Controller()
    renderer = Renderer(screen)

    elapsed_time = 0.0
    running = True

    while running:
        dt = clock.tick(FPS) / 1000.0
        if dt > 0.1: dt = 0.1

        for event in pygame.event.get():
            if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_r:
                    drone.reset()
                    controller.reset()
                    renderer.reset_trail()
                    elapsed_time = 0.0
                elif event.key == pygame.K_m:
                    controller.mode = "MANUAL" if controller.mode != "MANUAL" else "AUTOPILOT"

        if controller.mode == "AUTOPILOT" and not controller.course_completed:
            elapsed_time += dt

        keys = pygame.key.get_pressed()
        sensor.update(drone.x, drone.y, drone.yaw, world)
        throttle, steer = controller.compute(drone, sensor, world, keys)
        drone.set_inputs(throttle, steer)
        drone.update(dt)

        progress = world.calculate_progress(drone.x, drone.y)
        renderer.render(world, drone, sensor, controller, progress, elapsed_time, clock.get_fps())

    pygame.quit()


if __name__ == "__main__":
    main()
