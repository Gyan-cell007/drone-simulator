# Autonomous Drone Line Follower Simulator (Stage 1 Prototype)

Welcome to the **University Drone Competition Software Simulator** — Stage 1 Prototype.

This prototype features a fully autonomous 2D quadcopter drone simulation that follows a colored line on the ground from the **Start Pad** to the **Landing Pad** using simulated downward camera sensing and a simple PD controller.

---

## 📁 Project Directory Structure

```
drone_simulator/
├── config.py             # Global constants (screen size, physics limits, colors, waypoints, gains)
├── main.py               # Main simulation loop, real-time clock, and test runner
├── requirements.txt      # Python dependencies (pygame, numpy)
├── run.bat               # Windows double-click launcher
├── README.md             # Project documentation and guide
└── src/
    ├── __init__.py       # Package marker
    ├── drone.py          # Drone state, 2D kinematics, velocity model, and spinning rotor animation
    ├── world.py          # Course track geometry, waypoints, landing pads, and progress calculation
    ├── sensors.py        # Downward camera simulation, probe array, and lateral error computation
    ├── controller.py     # Autopilot PD line follower, speed modulation, and landing detection
    └── renderer.py       # Pygame graphics visualizer, flight trail breadcrumbs, and Telemetry HUD
```

---

## 🎮 Flight Telemetry & Controls

### On-Screen Telemetry HUD (Heads-Up Display)
During autonomous flight, the HUD displays:
- **Flight Mode**: `AUTOPILOT (LINE FOLLOWER)` or `LANDED (COURSE COMPLETE)`
- **Speed**: Real-time forward velocity in `px/s`
- **Lateral Error**: Distance in pixels from the camera center to the line's centroid (`[CENTERED]`, `[LINE IS LEFT]`, or `[LINE IS RIGHT]`)
- **Progress**: Travel percentage from `0.0%` to `100.0%` with a live visual progress bar
- **Elapsed Time**: Flight stopwatch in seconds (`mm:ss.s`)
- **Heading (Yaw)**: Orientation in degrees

### Keyboard Controls
| Key | Action |
| :--- | :--- |
| **`R`** | **Reset Flight Run**: Teleports drone back to the Start Pad, clears flight trail, and restarts timer |
| **`M`** | **Toggle Autopilot / Manual**: Switch between autonomous line follower and manual pilot controls |
| **`ESC`** | **Exit Simulation** |

*(In Manual Mode: `W`/`S` control thrust/brake, `A`/`D` steer left/right, and `SPACE` triggers emergency hover stop).*

---

## 🚀 How to Run the Simulator

### Option 1: Desktop Shortcut (Easiest)
Double-click **`Launch Drone Simulator.bat`** on your Desktop.

### Option 2: Project Folder Batch File
Double-click [`run.bat`](file:///C:/Users/manam/.gemini/antigravity/scratch/drone_simulator/run.bat) inside [`C:\Users\manam\.gemini\antigravity\scratch\drone_simulator`](file:///C:/Users/manam/.gemini/antigravity/scratch/drone_simulator).

### Option 3: Terminal Command
```powershell
& "$env:LOCALAPPDATA\Programs\Python\Python311\python.exe" "C:\Users\manam\.gemini\antigravity\scratch\drone_simulator\main.py"
```

### Option 4: Automated Verification Test
To run a fast headless self-test that verifies the entire flight from start to landing pad:
```powershell
& "$env:LOCALAPPDATA\Programs\Python\Python311\python.exe" "C:\Users\manam\.gemini\antigravity\scratch\drone_simulator\main.py" --test
```
