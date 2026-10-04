# Autonomous Drone Line-Following Simulator

An autonomous drone line-following simulator developed as a software prototype for the university drone competition. It provides a deterministic, two-dimensional Pygame environment for developing and exercising course following, route mapping, and saved-route navigation.

## Architecture

The simulator is organized around a small set of cooperating components:

- **Simulated drone** — 2D motion, heading, speed, and flight state.
- **World/course** — track geometry, start and landing pads, target line, and optional distractor lines.
- **Color-aware downward sensing** — samples the course beneath a simulated sensor bar and estimates lateral error from the configured target line.
- **Line-following controller** — steers and adjusts throttle from sensor observations, including a bounded response to line loss.
- **Path mapping** — records a flown route and reduces samples to compact waypoints for storage.
- **Known-path navigation** — follows a saved waypoint route while correcting against current sensor feedback.
- **Route similarity/change detection** — gathers route observations and checks whether the course matches the saved route before route guidance takes over.
- **Speed profiles and adaptive look-ahead** — adjust route speed and the forward guidance target for the route geometry and turn class.
- **Line-loss recovery** — briefly continues a controlled search using the last known line error, then commands a stop if the line remains unavailable.
- **Clean full-course mapping** — the `P` workflow resets the drone and mapping data, then records a new run from the configured start.
- **Renderer** — draws the course, drone, route/trail, sensing indicators, and flight/navigation telemetry.

Color-aware sensing is a deterministic simulation abstraction. The sensor samples configured course color data; it does **not** perform real camera capture, image processing, or real-world color recognition.

## Modes

- **AUTOPILOT** — follows the course line using the simulated sensor and line-following controller.
- **PATH_MAPPING** — follows the line while recording a route. The `P` key starts a clean mapping run; `M` can also cycle into mapping.
- **KNOWN_PATH** — checks observations against the saved route, then uses that route as its primary trajectory with sensor feedback for correction. A route mismatch returns the controller to ordinary line following.
- **MANUAL** — accepts pilot keyboard inputs.
- **LANDED** — indicates that the course has been completed and the drone has landed.

## Mapping and Known Path

Press **`P`** to reset to the configured course start, clear the current mapping buffer, and begin a clean full-course mapping run. The recorder compresses flight samples into compact waypoints. When the run reaches the landing pad, mapping is stopped and the route is saved as `saved_path.json`. Leaving mapping with `M`, resetting with `R`, or closing the simulator also saves an active recording.

`saved_path.json` is generated runtime data and is intentionally ignored by Git. A valid saved route is loaded on startup. Press **`K`** to enter Known Path mode. The controller first checks route similarity and gathers enough evidence before enabling route guidance. Once accepted, the route is the primary reference trajectory, with live sensor feedback providing correction. If the observed course differs from the saved route, Known Path guidance is rejected and normal line following continues. Pressing `K` while already in Known Path returns to AUTOPILOT.

## Controls

| Key | Action |
| --- | --- |
| `R` | Reset the drone to the start and clear the flight run; save any active mapping first. |
| `M` | Cycle between AUTOPILOT, MANUAL, PATH_MAPPING, and AUTOPILOT; entering mapping records a route. |
| `P` | Start a clean full-course mapping run from the configured start. |
| `K` | Enter Known Path using a valid saved route, or return to AUTOPILOT if already in Known Path. |
| `Esc` | Exit the simulator; an active mapping run is saved on close. |
| `W` / `Up` | Increase forward throttle in MANUAL. |
| `S` / `Down` | Apply reverse/braking input in MANUAL. |
| `A` / `Left` | Steer left in MANUAL. |
| `D` / `Right` | Steer right in MANUAL. |
| `Space` | Zero throttle and steering inputs in MANUAL. |

## Project Structure

```text
drone_simulator/
├── config.py             # Simulation, course, sensor, and controller settings
├── main.py               # Pygame entry point and headless verification commands
├── requirements.txt      # Python package dependencies
├── run.bat               # Windows launcher
├── saved_path.json       # Generated route data (ignored by Git; created at runtime)
├── standalone_sim.py     # Standalone simulation entry point
├── STAGE1_EXPLANATION.md # Earlier prototype notes
├── README.md
└── src/
    ├── __init__.py
    ├── controller.py      # Flight modes and navigation control
    ├── drone.py           # Drone state and simulated motion
    ├── path_manager.py    # Route recording, persistence, tracking, and comparison
    ├── renderer.py        # Pygame drawing and telemetry
    ├── sensors.py         # Downward target-line sensing
    └── world.py           # Course geometry and progress
```

## Setup and Run

Install Python, then from the project directory install the dependencies:

```sh
python -m pip install -r requirements.txt
```

Run the simulator with:

```sh
python main.py
```

On Windows, `run.bat` can also be used to launch the simulator. It runs `main.py` with the configured Python installation or falls back to `python` on `PATH`.

## Automated Verification

`main.py` provides these headless checks. Run one command at a time from the project directory:

```sh
python main.py --test
python main.py --test-mapping
python main.py --test-known-path
python main.py --test-route-similarity
python main.py --test-stage4-edge-cases
python main.py --test-clean-mapping
python main.py --test-stage5
python main.py --test-known-path-key
python main.py --test-line-loss
python main.py --test-color-sensing
```

## Limitations / Next Steps

This is currently a 2D Pygame simulation. It has not been validated on real drone hardware. Competition deployment will require adapting and validating the control and sensing approach for the competition's MATLAB/Simulink/PX4 environment and, eventually, real drone hardware.
