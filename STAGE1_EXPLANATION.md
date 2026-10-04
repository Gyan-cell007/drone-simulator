# University Drone Competition Prototype - Stage 1 Guide

## Overview
This document contains the complete architectural breakdown, line-detection theory, controller mathematics, and testing instructions for the Stage 1 Software Prototype.

---

## 1. Project Files Summary

| File | Language | Purpose | Required for Simulation? |
| :--- | :--- | :--- | :--- |
| **`config.py`** | Python | Central configuration for screen size, colors, drone limits, waypoints, and sensor gains. | **Yes** |
| **`main.py`** | Python | Main simulation execution loop, clock timing, event handling, and test harness. | **Yes** |
| **`src/drone.py`** | Python | 2D kinematic model of quadcopter (position, yaw, speed, rotor animation). | **Yes** |
| **`src/world.py`** | Python | Geometry of the course, amber track, start/finish pads, and progress calculation. | **Yes** |
| **`src/sensors.py`** | Python | Downward camera optical probe array and lateral error calculation. | **Yes** |
| **`src/controller.py`** | Python | Autopilot line-following PD algorithm, adaptive speed, and landing stop. | **Yes** |
| **`src/renderer.py`** | Python | Pygame drawing for world, drone, trail breadcrumbs, and live Telemetry HUD. | **Yes** |
| **`standalone_sim.py`** | Python | Single-file complete prototype combining all components in one file. | **Optional** |
| **`run.bat`** | Batch | Windows shortcut runner configured for Python 3.11. | **Optional** |
| **`requirements.txt`** | Text | Dependencies: `pygame`, `numpy`. | **Setup only** |

---

## 2. Line-Detection Logic Explained

### A. Lookahead Geometry
In real autonomous robotics, looking straight down directly at the drone's center causes delayed reactions because by the time the center crosses a curve, it is already too late to turn without overshooting.

Our downward camera projects a **sensor bar** at a lookahead distance $L = 32\text{ px}$ ahead of the drone:
$$\text{center}_x = \text{drone}_x + L \cdot \cos(\text{yaw})$$
$$\text{center}_y = \text{drone}_y + L \cdot \sin(\text{yaw})$$

### B. Optical Probe Array
Across this bar, we place $N = 25$ virtual photodetector probes spanning a width $W = 100\text{ px}$ perpendicular to the forward direction:
$$\text{probe}_i = \text{center} + s_i \cdot \vec{u}_{\text{right}}$$
where $s_i \in [-W/2, +W/2]$ is the lateral offset:
- Negative offsets = left of camera center
- Positive offsets = right of camera center

### C. Centroid and Lateral Error ($e$)
Each probe checks if it touches the amber path. If $K$ probes detect the path, the **line centroid** is the average of their offsets:
$$\text{lateral\_error } (e) = \frac{1}{K} \sum_{i=1}^K s_i$$

- **$e = 0.0\text{ px}$**: Line is dead center in front of the drone.
- **$e > 0.0\text{ px}$**: Line is to the **right** $\implies$ drone needs to steer right.
- **$e < 0.0\text{ px}$**: Line is to the **left** $\implies$ drone needs to steer left.

---

## 3. The Simple Controller Explained

The controller takes the measured lateral error ($e$) and calculates two physical outputs:
1. **`steering`** (turn rate: left vs right)
2. **`throttle`** (forward velocity)

### A. Steering via PD Control
$$\text{steering} = \underbrace{K_p \cdot e}_{\text{Proportional (P)}} + \underbrace{K_d \cdot (e - e_{\text{prev}})}_{\text{Derivative (D)}}$$

- **Proportional Term ($K_p = 0.035$):**  
  Corrects current displacement. The farther away the line is, the harder the drone turns toward it.
- **Derivative Term ($K_d = 0.20$):**  
  Acts as a damper against oscillation. When the drone turns and approaches the center line quickly, $(e - e_{\text{prev}})$ opposes the steering command, preventing the drone from weaving back and forth like a pendulum.
- **Clamping:** The final steering command is clamped to $[-1.0, 1.0]$.

### B. Speed (Throttle) Modulation
To negotiate turns safely without skidding wide:
- **Straightaways ($|e| < 12\text{ px}$):** Throttle is $0.72$ ($\approx 65\text{ px/s}$).
- **Gentle curves ($12 \le |e| < 25\text{ px}$):** Throttle reduces to $0.52$ ($\approx 47\text{ px/s}$).
- **Sharp turns ($|e| \ge 25\text{ px}$):** Throttle reduces to $0.38$ ($\approx 34\text{ px/s}$).
- **Landing:** When distance to the Landing Pad is within $30\text{ px}$ and course progress exceeds $85\%$, throttle cuts to $0.0$, bringing the drone to a stop inside the red circle.

---

## 4. Telemetry Definitions

The HUD continuously tracks four core metrics:
1. **Speed:** Current forward ground speed in pixels per second.
2. **Lateral Error:** Distance from the drone's centerline to the detected path center.
3. **Progress:** Traversed distance along the path as a percentage of total path length ($0.0\%$ to $100.0\%$).
4. **Elapsed Time:** Stopwatch timer showing active flight duration from takeoff to landing.
