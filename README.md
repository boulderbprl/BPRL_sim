# Multicopter Software-in-the-Loop (SITL) Simulation

A simulation framework for Software-in-the-Loop (SITL) testing of multicopter UAVs, supporting multiple vehicle platforms and controller architectures. This repository is used to develop, tune, and validate flight controllers for the **Hexsoon** and **BPRL** copter platforms in the lab.

---

## Table of Contents

- [Overview](#overview)
- [Supported Platforms](#supported-platforms)
- [Controller Architectures](#controller-architectures)
- [Prerequisites](#prerequisites)
- [Getting Started](#getting-started)
- [Directory Structure](#directory-structure)
- [Usage](#usage)
- [Contributing](#contributing)

---

## Overview

This repository provides a SITL simulation environment for multicopter UAVs. SITL allows the flight control software to run on a host computer — without hardware — by replacing the physical sensors and actuators with simulated equivalents. This enables rapid controller development, testing, and parameter tuning before deployment on real vehicles.

The framework supports three distinct controller architectures, ranging from the standard ArduPilot autopilot to fully custom MATLAB-based controllers with advanced inner-loop designs.

---

## Supported Platforms

| Platform | Type | Description |
|---|---|---|
| **Hexsoon EDU-450 / EDU-650** | Quadcopter / Hexacopter | Commercial multicopter frame used as primary development platform |
| **BPRL Copter** | Custom Multicopter | In-house lab platform used for experimental controller validation |

Vehicle parameters (mass, inertia, motor curves, propeller coefficients) for each platform are defined in their respective configuration files under `config/vehicles/`.

---

## Controller Architectures

Three flight controller architectures are implemented and can be selected at runtime.

### 1. ArduPilot Only
Uses the stock **ArduPilot** firmware running entirely within the SITL environment. This serves as the baseline controller and reference for performance comparison.

- Attitude and rate control handled natively by ArduPilot
- Full MAVLink interface for ground control station (GCS) connectivity
- Standard ArduCopter parameter tuning via `.param` files

### 2. Full MATLAB PID Controller
A complete replacement of the ArduPilot control stack with a **custom PID controller implemented in MATLAB/Simulink**. Both the outer loop (position/velocity) and inner loop (attitude/rate) are designed and tuned in MATLAB.

- Position controller → Velocity controller → Attitude controller → Rate controller
- All gains defined in MATLAB controller configuration scripts
- Interfaces with the simulated vehicle dynamics via the MATLAB/Simulink interface layer

### 3. Full MATLAB PID (Outer Loop) + INDI (Inner Loop)
A hybrid architecture combining a **MATLAB PID outer loop** for position and velocity tracking with an **Incremental Nonlinear Dynamic Inversion (INDI)** inner loop for attitude and angular rate control.

- Outer loop: PID-based position and velocity controller (identical to Architecture 2)
- Inner loop: INDI-based attitude and angular rate controller for improved disturbance rejection and robustness
- Actuator model and sensor data used directly in the INDI control law
- Suitable for aggressive manoeuvres and operation in turbulent conditions

---

## Prerequisites

- **MATLAB / Simulink** (R2022a or later recommended)
- **ArduPilot SITL** — [Setup guide](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html)
- **Python 3.8+** with `pymavlink`, `dronekit` (for MAVLink scripting)
- **QGroundControl** or **Mission Planner** (optional, for GCS)
- Linux / macOS recommended; WSL2 supported on Windows

Install Python dependencies:

```bash
pip install -r requirements.txt
```

---

## Getting Started

1. Clone the repository:

```bash
git clone https://github.com/<your-org>/<repo-name>.git
cd <repo-name>
```

2. Set up ArduPilot SITL (if not already installed):

```bash
cd ardupilot
./Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
```

3. Launch a simulation — see [Usage](#usage) below.

---

## Directory Structure

```
.
├── ardupilot/                      # ArduPilot SITL submodule / installation
│   └── Tools/
│       └── autotest/               # ArduPilot SITL scripts and vehicle configs
│
├── config/
│   ├── vehicles/
│   │   ├── hexsoon/                # Hexsoon vehicle parameters and airframe config
│   │   └── bprl_copter/            # BPRL copter parameters and airframe config
│   └── controllers/
│       ├── ardupilot/              # ArduPilot .param files for each platform
│       ├── matlab_pid/             # Gain schedules and config for full PID controller
│       └── matlab_pid_indi/        # Gain schedules and config for PID + INDI controller
│
├── controllers/
│   ├── ardupilot_only/             # Launch scripts and wrappers for ArduPilot SITL baseline
│   ├── matlab_pid/                 # Full MATLAB PID controller (outer + inner loop)
│   │   ├── outer_loop/             # Position and velocity PID controller (Simulink)
│   │   └── inner_loop/             # Attitude and rate PID controller (Simulink)
│   └── matlab_pid_indi/            # PID outer loop + INDI inner loop controller
│       ├── outer_loop/             # Position and velocity PID controller (Simulink)
│       └── inner_loop/             # INDI attitude and rate controller (Simulink)
│
├── dynamics/
│   ├── vehicle_model.m             # Multicopter rigid-body dynamics model
│   ├── motor_model.m               # Motor and ESC model
│   └── aero_model.m                # Aerodynamic disturbance model
│
├── interface/
│   ├── mavlink/                    # MAVLink interface layer (Python)
│   ├── matlab_sitl_bridge/         # MATLAB ↔ ArduPilot SITL UDP bridge
│   └── sensor_emulation/           # Simulated IMU, barometer, GPS outputs
│
├── missions/
│   ├── hover_test.py               # Basic hover validation mission
│   ├── waypoint_nav.py             # Waypoint following mission
│   └── step_response.py            # Step input tests for controller tuning
│
├── analysis/
│   ├── plot_logs.m                 # MATLAB log plotting and analysis scripts
│   ├── performance_metrics.m       # Controller performance metric calculations
│   └── compare_controllers.m       # Side-by-side comparison across architectures
│
├── logs/                           # Simulation log output (auto-generated, git-ignored)
│
├── docs/
│   ├── controller_design.md        # Theory and design notes for each controller
│   ├── vehicle_parameters.md       # Platform-specific parameter documentation
│   └── sitl_setup.md               # Detailed SITL environment setup guide
│
├── requirements.txt                # Python dependencies
├── .gitmodules                     # Git submodule config (ArduPilot)
└── README.md
```

---

## Usage

### ArduPilot Only (Baseline)

```bash
# Launch SITL with Hexsoon airframe
cd ardupilot
sim_vehicle.py -v ArduCopter --frame=hexa \
  --add-param-file=../config/vehicles/hexsoon/hexsoon.param

# Or BPRL copter
sim_vehicle.py -v ArduCopter --frame=quad \
  --add-param-file=../config/vehicles/bprl_copter/bprl.param
```

### Full MATLAB PID Controller

1. Open MATLAB and navigate to `controllers/matlab_pid/`
2. Run the initialisation script for the target platform:

```matlab
init_hexsoon   % or init_bprl
```

3. Open and run the Simulink model:

```matlab
sim('multicopter_pid_sitl.slx')
```

### PID + INDI Controller

1. Open MATLAB and navigate to `controllers/matlab_pid_indi/`
2. Run the initialisation script:

```matlab
init_hexsoon_indi   % or init_bprl_indi
```

3. Open and run the Simulink model:

```matlab
sim('multicopter_pid_indi_sitl.slx')
```

---

## Contributing

1. Fork the repository and create a feature branch: `git checkout -b feature/my-change`
2. Follow the existing code and documentation structure
3. Test changes on both vehicle platforms before submitting a pull request
4. Include relevant log files or plots if adding or modifying controller logic

---

## License

[MIT License](LICENSE) — see `LICENSE` for details.

---

*Developed by the BPRL UAV Research Group.*