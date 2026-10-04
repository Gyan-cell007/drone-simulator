"""
renderer.py
=============================================================================
Pygame Visualizer: Renders the world, drone, trail, sensors, and telemetry HUD.

For beginners:
- Handles drawing the 2D environment cleanly separated from physics and control.
- Displays:
    1. Grid floor, Start Pad, and Landing Pad
    2. Coloured amber path
    3. Breadcrumb flight trail showing the drone's historical trajectory
    4. Quadcopter body with spinning rotors and forward indicator
    5. Sensor probe bar with green active detection dots
    6. Telemetry HUD with Speed, Lateral Error, Progress %, and Elapsed Time
=============================================================================
"""

import math
import pygame
from typing import List, Tuple
from config import (
    WINDOW_WIDTH,
    WINDOW_HEIGHT,
    COLOR_BG,
    COLOR_GRID,
    COLOR_PATH_BORDER,
    COLOR_START_PAD,
    COLOR_LAND_PAD,
    COLOR_DRONE_BODY,
    COLOR_DRONE_NOSE,
    COLOR_ROTOR_ARM,
    COLOR_ROTOR_BLADE,
    COLOR_SENSOR_LINE,
    COLOR_PROBE_ACTIVE,
    COLOR_PROBE_IDLE,
    COLOR_ERROR_INDICATOR,
    COLOR_TRAIL,
    COLOR_MAPPED_PATH,
    COLOR_MAPPED_NODE,
    COLOR_TEXT,
    COLOR_TEXT_DIM,
    COLOR_HUD_BG,
)
from src.drone import Drone
from src.world import World
from src.sensors import LineSensor


class Renderer:
    """
    Renders the complete 2D simulation canvas.
    """

    def __init__(self, screen: pygame.Surface):
        self.screen = screen
        self.width = WINDOW_WIDTH
        self.height = WINDOW_HEIGHT

        # Historical flight trail breadcrumbs
        self.trail: List[Tuple[float, float]] = []
        self.max_trail_length = 1200

        # Initialize fonts
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
        """Clear flight breadcrumbs when resetting."""
        self.trail.clear()

    def record_trail(self, x: float, y: float):
        """Append current drone position to flight path trail."""
        if not self.trail or math.hypot(x - self.trail[-1][0], y - self.trail[-1][1]) > 3.0:
            self.trail.append((x, y))
            if len(self.trail) > self.max_trail_length:
                self.trail.pop(0)

    def draw_grid(self):
        """Draw background floor grid lines."""
        grid_size = 50
        for x in range(0, self.width, grid_size):
            pygame.draw.line(self.screen, COLOR_GRID, (x, 0), (x, self.height), 1)
        for y in range(0, self.height, grid_size):
            pygame.draw.line(self.screen, COLOR_GRID, (0, y), (self.width, y), 1)

    def draw_world(self, world: World):
        """Draw the competition course: colored track and landing pads."""
        # 1. Path Border / Outline
        for i in range(len(world.waypoints) - 1):
            p1 = world.waypoints[i]
            p2 = world.waypoints[i + 1]
            pygame.draw.line(self.screen, COLOR_PATH_BORDER, p1, p2, int(world.line_width) + 4)
            pygame.draw.circle(self.screen, COLOR_PATH_BORDER, (int(p1[0]), int(p1[1])), int(world.line_width // 2) + 2)
            pygame.draw.circle(self.screen, COLOR_PATH_BORDER, (int(p2[0]), int(p2[1])), int(world.line_width // 2) + 2)

        # 2. Main Colored Line
        for i in range(len(world.waypoints) - 1):
            p1 = world.waypoints[i]
            p2 = world.waypoints[i + 1]
            pygame.draw.line(self.screen, world.target_line_color, p1, p2, int(world.line_width))
            pygame.draw.circle(self.screen, world.target_line_color, (int(p1[0]), int(p1[1])), int(world.line_width // 2))
            pygame.draw.circle(self.screen, world.target_line_color, (int(p2[0]), int(p2[1])), int(world.line_width // 2))

        # Draw distractors above the target just as sample_line_color overlays
        # their color where line strokes intersect.
        for distractor in world.distractor_lines:
            points = distractor["waypoints"]
            color = distractor["color"]
            width = int(distractor["line_width"])
            for i in range(len(points) - 1):
                p1, p2 = points[i], points[i + 1]
                pygame.draw.line(self.screen, color, p1, p2, width)
                pygame.draw.circle(self.screen, color, (int(p1[0]), int(p1[1])), width // 2)
                pygame.draw.circle(self.screen, color, (int(p2[0]), int(p2[1])), width // 2)

        # 3. Start Pad (Green circle with target rings)
        sx, sy = int(world.start_pos[0]), int(world.start_pos[1])
        pygame.draw.circle(self.screen, (20, 90, 50), (sx, sy), int(world.pad_radius) + 4)
        pygame.draw.circle(self.screen, COLOR_START_PAD, (sx, sy), int(world.pad_radius))
        pygame.draw.circle(self.screen, (255, 255, 255), (sx, sy), int(world.pad_radius * 0.7), 2)
        txt_start = self.font_pad.render("START", True, (255, 255, 255))
        self.screen.blit(txt_start, (sx - txt_start.get_width() // 2, sy - txt_start.get_height() // 2))

        # 4. Landing Pad (Red circle with target rings)
        lx, ly = int(world.landing_pos[0]), int(world.landing_pos[1])
        pygame.draw.circle(self.screen, (120, 30, 30), (lx, ly), int(world.pad_radius) + 4)
        pygame.draw.circle(self.screen, COLOR_LAND_PAD, (lx, ly), int(world.pad_radius))
        pygame.draw.circle(self.screen, (255, 255, 255), (lx, ly), int(world.pad_radius * 0.7), 2)
        txt_land = self.font_pad.render("LAND", True, (255, 255, 255))
        self.screen.blit(txt_land, (lx - txt_land.get_width() // 2, ly - txt_land.get_height() // 2))

    def draw_trail(self):
        """Draw cyan breadcrumb flight path trail behind the drone."""
        if len(self.trail) > 1:
            pygame.draw.lines(self.screen, COLOR_TRAIL, False, self.trail, 2)

    def draw_mapped_path(self, mapped_waypoints: list):
        """
        Draw the compact recorded/saved route as a distinct visual overlay.
        Shows connecting route line in bright magenta with distinct waypoint nodes.
        """
        if not mapped_waypoints:
            return

        points = [(int(wp["x"]), int(wp["y"])) for wp in mapped_waypoints]

        # Draw connecting line segments between saved waypoints
        if len(points) > 1:
            pygame.draw.lines(self.screen, COLOR_MAPPED_PATH, False, points, 2)

        # Draw individual waypoint node circles
        for i, (px, py) in enumerate(points):
            # Normal node dot
            pygame.draw.circle(self.screen, COLOR_MAPPED_NODE, (px, py), 3)
            pygame.draw.circle(self.screen, (20, 25, 35), (px, py), 1)

        # Highlight Start and End mapped waypoints with distinct rings
        if points:
            # Start mapped waypoint
            pygame.draw.circle(self.screen, (46, 204, 113), points[0], 6, 2)
            # End mapped waypoint
            pygame.draw.circle(self.screen, (235, 77, 180), points[-1], 6, 2)

    def draw_lookahead_target(self, target):
        """Mark the active route-following target on the course."""
        if target is None:
            return
        point = (int(target["x"]), int(target["y"]))
        pygame.draw.circle(self.screen, COLOR_ERROR_INDICATOR, point, 9, 2)
        pygame.draw.line(self.screen, COLOR_ERROR_INDICATOR, (point[0] - 5, point[1]), (point[0] + 5, point[1]), 1)
        pygame.draw.line(self.screen, COLOR_ERROR_INDICATOR, (point[0], point[1] - 5), (point[0], point[1] + 5), 1)

    def draw_sensor(self, drone: Drone, sensor: LineSensor):
        """Draw the downward-facing camera FOV bar and virtual probe detectors."""
        yaw_rad = math.radians(drone.yaw)
        fwd_x, fwd_y = math.cos(yaw_rad), math.sin(yaw_rad)

        cx = drone.x + fwd_x * sensor.lookahead
        cy = drone.y + fwd_y * sensor.lookahead

        # Lookahead connector ray
        pygame.draw.line(self.screen, COLOR_SENSOR_LINE, (drone.x, drone.y), (cx, cy), 1)

        # Draw sensor bar baseline
        if sensor.probes_data:
            left_probe = sensor.probes_data[0]
            right_probe = sensor.probes_data[-1]
            pygame.draw.line(
                self.screen,
                (70, 90, 110),
                (left_probe["x"], left_probe["y"]),
                (right_probe["x"], right_probe["y"]),
                2,
            )

        # Draw individual probe sample dots
        for probe in sensor.probes_data:
            px, py = int(probe["x"]), int(probe["y"])
            if probe["detected"]:
                pygame.draw.circle(self.screen, COLOR_PROBE_ACTIVE, (px, py), 4)
                pygame.draw.circle(self.screen, (255, 255, 255), (px, py), 1)
            else:
                pygame.draw.circle(self.screen, COLOR_PROBE_IDLE, (px, py), 2)

        # Draw centroid marker (where the line center is detected)
        if sensor.line_detected:
            right_x = -fwd_y
            right_y = fwd_x
            error_x = int(cx + right_x * sensor.lateral_error)
            error_y = int(cy + right_y * sensor.lateral_error)

            pygame.draw.circle(self.screen, COLOR_ERROR_INDICATOR, (error_x, error_y), 6)
            pygame.draw.circle(self.screen, (0, 0, 0), (error_x, error_y), 7, 1)

    def draw_drone(self, drone: Drone):
        """Draw the quadcopter drone with 4 arms, spinning rotors, and heading arrow."""
        dx, dy = drone.x, drone.y
        yaw_rad = math.radians(drone.yaw)
        arm_len = drone.radius * 1.3

        rotor_angles = [math.pi / 4, 3 * math.pi / 4, 5 * math.pi / 4, 7 * math.pi / 4]

        # Draw arms from drone center to each motor
        for r_ang in rotor_angles:
            angle = yaw_rad + r_ang
            rx = dx + arm_len * math.cos(angle)
            ry = dy + arm_len * math.sin(angle)
            pygame.draw.line(self.screen, COLOR_ROTOR_ARM, (dx, dy), (rx, ry), 3)
            pygame.draw.circle(self.screen, (40, 50, 60), (int(rx), int(ry)), 5)

            # Draw spinning propeller blades
            blade_r = 9.0
            spin_rad = math.radians(drone.rotor_angle)
            bx1 = rx + blade_r * math.cos(spin_rad)
            by1 = ry + blade_r * math.sin(spin_rad)
            bx2 = rx - blade_r * math.cos(spin_rad)
            by2 = ry - blade_r * math.sin(spin_rad)
            pygame.draw.line(self.screen, COLOR_ROTOR_BLADE, (bx1, by1), (bx2, by2), 2)
            pygame.draw.circle(self.screen, (220, 230, 240, 100), (int(rx), int(ry)), int(blade_r), 1)

        # Central avionics fuselage
        pygame.draw.circle(self.screen, (25, 30, 40), (int(dx), int(dy)), int(drone.radius) + 1)
        pygame.draw.circle(self.screen, COLOR_DRONE_BODY, (int(dx), int(dy)), int(drone.radius))
        pygame.draw.circle(self.screen, (255, 255, 255), (int(dx), int(dy)), 4)

        # Forward heading nose indicator (orange triangle)
        nose_len = drone.radius * 1.4
        nose_tip = (dx + nose_len * math.cos(yaw_rad), dy + nose_len * math.sin(yaw_rad))
        left_corner = (
            dx + drone.radius * 0.7 * math.cos(yaw_rad + 2.5),
            dy + drone.radius * 0.7 * math.sin(yaw_rad + 2.5),
        )
        right_corner = (
            dx + drone.radius * 0.7 * math.cos(yaw_rad - 2.5),
            dy + drone.radius * 0.7 * math.sin(yaw_rad - 2.5),
        )
        pygame.draw.polygon(self.screen, COLOR_DRONE_NOSE, [nose_tip, left_corner, right_corner])

    def draw_hud(
        self,
        drone: Drone,
        sensor: LineSensor,
        mode_str: str,
        progress: float,
        elapsed_time: float,
        fps: float,
        mapping_status: str,
        waypoint_count: int,
        known_path_state: str,
        route_progress: float,
        route_distance: float,
        similarity_score: float,
        similarity_confidence: float,
        similarity_decision: str,
    ):
        """
        Draw the real-time Telemetry HUD with:
        - Speed
        - Lateral Error
        - Progress
        - Elapsed Time
        """
        hud_w, hud_h = 370, 345
        hud_surface = pygame.Surface((hud_w, hud_h), pygame.SRCALPHA)
        hud_surface.fill(COLOR_HUD_BG)
        pygame.draw.rect(hud_surface, (70, 80, 100), (0, 0, hud_w, hud_h), 1)

        # Header Title
        title_surf = self.font_title.render("DRONE TELEMETRY HUD (STAGE 1)", True, (245, 176, 65))
        hud_surface.blit(title_surf, (14, 10))

        # Format lateral error text
        err_text = "N/A (Line Lost)"
        err_color = (231, 76, 60)
        if sensor.line_detected:
            if abs(sensor.lateral_error) < 2.0:
                err_text = f"{sensor.lateral_error:+.1f} px [CENTERED]"
                err_color = (46, 204, 113)
            elif sensor.lateral_error < 0:
                err_text = f"{sensor.lateral_error:+.1f} px [LINE IS LEFT]"
                err_color = (52, 152, 219)
            else:
                err_text = f"{sensor.lateral_error:+.1f} px [LINE IS RIGHT]"
                err_color = (230, 126, 34)

        # Format elapsed time as mm:ss.s
        minutes = int(elapsed_time // 60)
        seconds = elapsed_time % 60
        time_str = f"{minutes:02d}:{seconds:04.1f} s"

        lines = [
            ("Flight Mode:", f"{mode_str}", (255, 255, 255)),
            ("Path Mapping:", f"{mapping_status} ({waypoint_count})", COLOR_MAPPED_PATH),
            ("Known Path:", known_path_state, COLOR_ERROR_INDICATOR),
            ("Route Match:", f"{similarity_score:.2f} (conf {similarity_confidence:.2f})", COLOR_MAPPED_PATH),
            ("Match Decision:", similarity_decision, COLOR_TEXT),
            ("Route Progress:", f"{route_progress:5.1f} %", COLOR_TEXT),
            ("Route Distance:", f"{route_distance:5.1f} px", COLOR_TEXT_DIM),
            ("Speed:", f"{drone.speed:.1f} px/s", (100, 220, 255)),
            ("Lateral Error:", err_text, err_color),
            ("Progress:", f"{progress:5.1f} %", (46, 204, 113) if progress >= 99.0 else COLOR_TEXT),
            ("Elapsed Time:", time_str, (241, 196, 15)),
            ("Heading (Yaw):", f"{drone.yaw:.1f}°", COLOR_TEXT),
            ("Active Probes:", f"{sensor.active_probe_count}/{sensor.num_probes}", COLOR_TEXT_DIM),
            ("Simulation Rate:", f"{fps:.0f} FPS", COLOR_TEXT_DIM),
        ]

        y_offset = 36
        for label, val, color in lines:
            lbl_surf = self.font_main.render(label, True, COLOR_TEXT_DIM)
            val_surf = self.font_bold.render(val, True, color)
            hud_surface.blit(lbl_surf, (14, y_offset))
            hud_surface.blit(val_surf, (140, y_offset))
            y_offset += 20

        # Draw visual progress bar at bottom of HUD
        bar_x, bar_y, bar_w, bar_h = 14, y_offset + 5, hud_w - 28, 10
        pygame.draw.rect(hud_surface, (40, 45, 55), (bar_x, bar_y, bar_w, bar_h), border_radius=3)
        fill_w = int((progress / 100.0) * bar_w)
        if fill_w > 0:
            pygame.draw.rect(hud_surface, (46, 204, 113), (bar_x, bar_y, fill_w, bar_h), border_radius=3)
        pygame.draw.rect(hud_surface, (90, 100, 120), (bar_x, bar_y, bar_w, bar_h), 1, border_radius=3)

        self.screen.blit(hud_surface, (15, 15))

        # Bottom Instructions Panel
        ctrl_w, ctrl_h = 420, 72
        ctrl_surface = pygame.Surface((ctrl_w, ctrl_h), pygame.SRCALPHA)
        ctrl_surface.fill(COLOR_HUD_BG)
        pygame.draw.rect(ctrl_surface, (70, 80, 100), (0, 0, ctrl_w, ctrl_h), 1)

        c_title = self.font_bold.render("CONTROLS", True, (245, 176, 65))
        c_line1 = self.font_main.render("[R] Reset Flight Run     [M] Manual/Mapping cycle", True, COLOR_TEXT)
        c_line2 = self.font_main.render("[P] Start Clean Full-Course Mapping", True, COLOR_TEXT)
        c_line3 = self.font_main.render("[ESC] Exit", True, COLOR_TEXT_DIM)

        ctrl_surface.blit(c_title, (12, 6))
        ctrl_surface.blit(c_line1, (12, 23))
        ctrl_surface.blit(c_line2, (12, 40))
        ctrl_surface.blit(c_line3, (12, 57))
        self.screen.blit(ctrl_surface, (self.width - ctrl_w - 15, self.height - ctrl_h - 15))

    def draw_completion_banner(self, elapsed_time: float):
        """Draw celebration banner when the drone successfully reaches and lands at the finish."""
        banner_w, banner_h = 520, 85
        bx = (self.width - banner_w) // 2
        by = 25

        banner_surf = pygame.Surface((banner_w, banner_h), pygame.SRCALPHA)
        banner_surf.fill((20, 30, 25, 230))
        pygame.draw.rect(banner_surf, (46, 204, 113), (0, 0, banner_w, banner_h), 2, border_radius=8)

        t1 = self.font_banner.render("COURSE COMPLETED! LANDED AT TARGET", True, (46, 204, 113))
        t2 = self.font_main.render(f"Course Completed in {elapsed_time:.2f} seconds! Press [R] to Reset Run", True, (240, 240, 240))

        banner_surf.blit(t1, (banner_w // 2 - t1.get_width() // 2, 14))
        banner_surf.blit(t2, (banner_w // 2 - t2.get_width() // 2, 48))

        self.screen.blit(banner_surf, (bx, by))

    def render(
        self,
        world: World,
        drone: Drone,
        sensor: LineSensor,
        mode_str: str,
        progress: float,
        elapsed_time: float,
        is_completed: bool,
        fps: float,
        mapped_waypoints: list,
        mapping_status: str,
        known_path_state: str,
        route_progress: float,
        route_distance: float,
        lookahead_target,
        similarity_score: float,
        similarity_confidence: float,
        similarity_decision: str,
    ):
        """Execute one complete frame render."""
        self.record_trail(drone.x, drone.y)
        self.screen.fill(COLOR_BG)
        self.draw_grid()
        self.draw_world(world)
        self.draw_trail()
        self.draw_mapped_path(mapped_waypoints)
        self.draw_lookahead_target(lookahead_target)
        self.draw_sensor(drone, sensor)
        self.draw_drone(drone)
        self.draw_hud(
            drone,
            sensor,
            mode_str,
            progress,
            elapsed_time,
            fps,
            mapping_status,
            len(mapped_waypoints),
            known_path_state,
            route_progress,
            route_distance,
            similarity_score,
            similarity_confidence,
            similarity_decision,
        )

        if is_completed:
            self.draw_completion_banner(elapsed_time)

        pygame.display.flip()
