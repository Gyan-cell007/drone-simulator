"""
config.py
=============================================================================
Configuration settings and constants for the Autonomous Drone Simulator.

This file centralizes all tunable parameters (window size, colors, drone limits,
sensor properties, and default track waypoints) so that you do not need to hunt
for magic numbers inside the simulation code.
=============================================================================
"""

# ---------------------------------------------------------------------------
# 1. DISPLAY & SIMULATION SETTINGS
# ---------------------------------------------------------------------------
WINDOW_WIDTH = 1000
WINDOW_HEIGHT = 700
FPS = 60
SIM_TITLE = "Autonomous Drone Simulator - Stage 1: Line Following Prototype"

# ---------------------------------------------------------------------------
# 2. COLOR DEFINITIONS (RGB)
# ---------------------------------------------------------------------------
# Background and UI
COLOR_BG = (24, 26, 32)            # Dark slate gray
COLOR_GRID = (36, 40, 50)          # Subtle grid lines
COLOR_TEXT = (240, 240, 240)       # Crisp white
COLOR_TEXT_DIM = (160, 165, 175)   # Light gray for secondary text
COLOR_HUD_BG = (15, 17, 22, 215)   # Semi-transparent dark background for HUD

# Course & Track
TARGET_LINE_COLOR = (245, 176, 65) # Simulated color identity of the intended line
COLOR_PATH = TARGET_LINE_COLOR    # Renderer compatibility alias
COLOR_PATH_BORDER = (195, 135, 30) # Path edge contour
DEFAULT_DISTRACTOR_LINE_COLOR = (70, 150, 230)
COLOR_START_PAD = (46, 204, 113)   # Green starting pad
COLOR_LAND_PAD = (231, 76, 60)     # Red landing / target pad

# Drone Body & Animation
COLOR_DRONE_BODY = (41, 128, 185)  # Metallic blue drone frame
COLOR_DRONE_NOSE = (230, 126, 34)  # Orange forward indicator arrow
COLOR_ROTOR_ARM = (127, 140, 141)  # Quadcopter carbon arms
COLOR_ROTOR_BLADE = (200, 214, 229)# Spinning propeller blades
COLOR_TRAIL = (52, 152, 219)       # Cyan trail line behind drone
COLOR_MAPPED_PATH = (235, 77, 180)  # Bright magenta for recorded/saved route
COLOR_MAPPED_NODE = (255, 140, 230) # Waypoint marker nodes on mapped route

# Sensor Visualization
COLOR_SENSOR_LINE = (52, 152, 219) # Sensor lookahead ray
COLOR_PROBE_ACTIVE = (46, 204, 113)# Probe detects the line (Green)
COLOR_PROBE_IDLE = (120, 120, 120) # Probe detects floor only (Gray)
COLOR_ERROR_INDICATOR = (241, 196, 15) # Centroid/Lateral error mark (Yellow)

# ---------------------------------------------------------------------------
# PATH MAPPING & STORAGE CONSTANTS (STAGE 2)
# ---------------------------------------------------------------------------
SAVED_PATH_FILE = "saved_path.json" # Default filename for saved routes
MAP_SAMPLE_MIN_DIST = 16.0          # Minimum distance (pixels) to sample a new waypoint
MAP_SAMPLE_MIN_YAW_DEG = 7.0        # Minimum heading change (degrees) to sample
MAP_SAMPLE_MIN_TIME_SEC = 0.45      # Minimum elapsed time (seconds) to force sample

# Known-path tracking
KNOWN_PATH_LOOKAHEAD_DIST = 48.0
KNOWN_PATH_MAX_DISTANCE = 65.0
KNOWN_PATH_SENSOR_CORRECTION_LIMIT = 0.20
KNOWN_PATH_HEADING_GAIN_DEG = 55.0
KNOWN_PATH_SEARCH_AHEAD_SEGMENTS = 12

# Known Path speed profile (speed units are pixels per second).
KNOWN_PATH_MIN_SPEED = 24.0
KNOWN_PATH_MAX_SPEED = 80.0
KNOWN_PATH_STRAIGHT_SPEED = 75.0
KNOWN_PATH_GENTLE_CURVE_SPEED = 62.0
KNOWN_PATH_SHARP_TURN_SPEED = 48.0
KNOWN_PATH_GENTLE_TURN_THRESHOLD_DEG = 3.0
KNOWN_PATH_SHARP_TURN_THRESHOLD_DEG = 7.0
KNOWN_PATH_SHARP_TURN_CLUSTER_DIST = 70.0
KNOWN_PATH_SHARP_TURN_MIN_VERTICES = 3
KNOWN_PATH_SPEED_LOOKAHEAD_DIST = 110.0
KNOWN_PATH_MIN_LOOKAHEAD_DIST = 28.0
KNOWN_PATH_MAX_LOOKAHEAD_DIST = 52.0
KNOWN_PATH_GENTLE_LOOKAHEAD_SCALE = 0.84
KNOWN_PATH_SHARP_LOOKAHEAD_SCALE = 0.62
KNOWN_PATH_ACCEL_LIMIT = 32.0       # Pixels per second squared
KNOWN_PATH_DECEL_LIMIT = 58.0       # Pixels per second squared

# Online comparison of sensor-observed course geometry with the saved route
ROUTE_SIMILARITY_ACCEPT_THRESHOLD = 0.82
ROUTE_SIMILARITY_UNCERTAIN_THRESHOLD = 0.58
ROUTE_SIMILARITY_CHANGED_THRESHOLD = 0.35
ROUTE_SIMILARITY_MIN_SAMPLES = 8
# Require evidence across a meaningful portion of the route, not just its
# opening segment, before allowing Known Path guidance.
ROUTE_SIMILARITY_MIN_ROUTE_COVERAGE = 0.35
ROUTE_SIMILARITY_SAMPLE_DISTANCE = 8.0
ROUTE_SIMILARITY_POSITION_TOLERANCE = 10.0
ROUTE_SIMILARITY_POSITION_SCALE = 24.0
ROUTE_SIMILARITY_HEADING_SCALE_DEG = 40.0
ROUTE_SIMILARITY_ANCHOR_DISTANCE = 48.0
ROUTE_SIMILARITY_ANCHOR_TIMEOUT_DISTANCE = 240.0

# ---------------------------------------------------------------------------
# 3. DRONE PHYSICAL LIMITS & DYNAMICS
# ---------------------------------------------------------------------------
DRONE_RADIUS = 20.0                 # Physical footprint radius (pixels)
MAX_SPEED = 180.0                   # Max forward velocity (pixels / second)
ACCELERATION = 140.0                # Forward acceleration (pixels / second^2)
DECELERATION = 180.0                # Natural braking friction (pixels / second^2)
MAX_TURN_RATE = 160.0               # Max rotation speed (degrees / second)

# ---------------------------------------------------------------------------
# 4. DOWNWARD CAMERA / LINE SENSOR SPECIFICATIONS
# ---------------------------------------------------------------------------
SENSOR_LOOKAHEAD_DIST = 32.0        # Pixels ahead of the drone center
SENSOR_SPAN_WIDTH = 100.0           # Lateral field of view width (pixels)
SENSOR_NUM_PROBES = 25              # Number of virtual photodetector sample points

# ---------------------------------------------------------------------------
# 5. AUTOPILOT CONTROLLER PARAMETERS (STAGE 1)
# ---------------------------------------------------------------------------
AUTOPILOT_CRUISE_SPEED = 90.0       # Target forward cruise speed (pixels / sec)
AUTOPILOT_KP = 0.040                # Proportional gain: steering response to lateral error
AUTOPILOT_KD = 0.025                # Derivative gain: damping response to rate of error change
AUTOPILOT_LINE_LOSS_RECOVERY_DURATION = 0.75  # Seconds to search before stopping
AUTOPILOT_LINE_LOSS_RECOVERY_THROTTLE = 0.30   # Existing reduced throttle during search

# ---------------------------------------------------------------------------
# 6. DEFAULT TRACK & COURSE WAYPOINTS
# ---------------------------------------------------------------------------
PATH_LINE_WIDTH = 18                # Thickness of the colored path on the floor

# Smooth competition course with straights, gentle turns, and sweeping curves
DEFAULT_TRACK_WAYPOINTS = [
    (120, 560),   # Start Pad
    (260, 560),   # Initial straight section
    (380, 510),   # Gentle turn
    (480, 410),   # Ascending sweeping curve
    (560, 300),   # Curve upward
    (660, 240),   # Entering high-speed crest
    (760, 240),   # Top straight section
    (840, 310),   # Descending curve
    (880, 400),   # Right-side turn
    (880, 480),   # Hairpin turn around
    (830, 540),   # Re-entering bottom straight
    (740, 560),   # Straightening out
    (640, 560),   # Final approach section
    (520, 560)    # Landing Pad
]
