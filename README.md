# Vision-Guided Drone Precision Landing (Windows / C++)

**Important: This repository is a staged implementation, not a completed integrated ArduPilot precision landing system.** The working pieces are C++ PID kinematic simulation, OpenCV ArUco marker detection, marker pose estimation (from a supplied calibrated camera), checkerboard calibration, latency logs, repeated trial analysis and tests. True closed-loop drone flight with ArduPilot SITL and MAVLink remains integration work.

## Setup (Windows UCRT64 MSYS2)

Open **MSYS2 UCRT64** and install:

```bash
pacman -S --needed mingw-w64-ucrt-x86_64-gcc mingw-w64-ucrt-x86_64-cmake mingw-w64-ucrt-x86_64-ninja mingw-w64-ucrt-x86_64-opencv
```

From the project folder in the same terminal:

```bash
cmake -S . -B build -G Ninja
cmake --build build
./build/drone.exe demo 30
./build/drone.exe marker
ctest --test-dir build --output-on-failure
```

Or double-click `run.bat` after adjusting its MSYS2 path if necessary.

For optional charts install Python matplotlib in a separate Windows Python virtual environment:

```powershell
python -m venv .venv
.venv\Scripts\python -m pip install matplotlib
.venv\Scripts\python scripts\plot_results.py
```

## Camera calibration

Print a chessboard with **9 x 6 inner corners** (10 x 7 squares), with known square size; capture **at least 8 images** at varied positions and angles. Store them under `calibration/` (create directory). Then:

```bash
./build/drone.exe calibrate "calibration/*.jpg" config/camera.yaml 9 6 0.025
```

Change 0.025 if your real square side is not 25 mm. The example file is intentionally NOT a fake calibration. To detect a marker from a photo:

```bash
./build/drone.exe marker
./build/drone.exe detect your_picture.jpg config/camera.yaml 0.15
./build/drone.exe webcam config/camera.yaml 0.15
```

The marker size 0.15 means a physical marker with side length 15 cm. `results/marker.png` must be printed at a **measured** size before pose estimates can be meaningful. Camera calibration must match the camera used to take images.

## File/output guide

- `src/vision.cpp`: ArUco detection, pose estimation using OpenCV `solvePnP`, chessboard calibration.
- `src/landing.cpp`: 2D kinematic motion, PID, noise, dropped image measurements, target displacement.
- `results/trials.csv`: final position error and test summaries for repeated runs.
- `results/trajectory.csv`: time-series for first trial.
- `results/vision_latency.csv`: marker processing time for webcam mode.
- `scripts/plot_results.py`: error distribution and time series plots.
- `tests/test_pid.cpp`: basic controller regression checks.

## Not yet implemented / honest résumé claims

| Requirement | State |
|--|--|
| C++ pipeline + OpenCV ArUco detection | Implemented; requires compiling/running on your PC |
| Calibration + target-relative pose | Implemented; requires real checkerboard pictures / calibrated camera to test |
| PID-controlled simulated alignment | Implemented in **simple 2D kinematic model**, not aircraft dynamics |
| Gain tuning, overshoot, settling time, target displacement | Parameters and measurement are present; automated gain sweep is a future enhancement |
| Noise + frame drops, repeated error distributions | Implemented in simplified position-measurement simulator |
| Perception timing | Implemented; performance must be measured on your machine |
| Memory allocation profiling | Not implemented yet |
| ArduPilot SITL / MAVLink telemetry integration | **Not implemented yet** |
| Linux performance profiling | Not done; Windows requested |

## SITL integration design (next phase)

1. Install a supported Windows ArduPilot SITL build and compatible simulator, validate manual flight in a safe virtual world.
2. Install `pymavlink` into a dedicated Python environment to develop and log a telemetry bridge, or use a compatible C++ MAVLink library.
3. Feed timestamped camera-relative marker pose into an appropriately configured simulator/autopilot interface, validating coordinate transforms, timestamps and estimator state.
4. Implement guarded guidance and state transitions, verify flight-mode permissions and emergency stop in SITL only.
5. Log autopilot position, commanded velocity, observed image position and landing error. Compare to 2D baseline and report genuine SITL metrics only after working integration.

No flying a physical drone is assumed or recommended from this prototype.
