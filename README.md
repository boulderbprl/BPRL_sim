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
| **Hexsoon EDU-450 ** | Quadcopter / Hexacopter | Simple Multicopter Platform used as Example |
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
A complete replacement of the ArduPilot control stack with a **custom PID controller implemented in programmatic MATLAB**. Both the outer loop (position/velocity) and inner loop (attitude/rate) are designed and tuned in MATLAB.

- Position controller → Velocity controller → Attitude controller → Rate controller
- All gains defined in MATLAB controller configuration scripts
- Interfaces with the simulated vehicle dynamics 

### 3. Full MATLAB PID (Outer Loop) + INDI (Inner Loop)
A hybrid architecture combining a **MATLAB PID outer loop** for position and velocity tracking with an **Incremental Nonlinear Dynamic Inversion (INDI)** inner loop for attitude and angular rate control.

- Outer loop: PID-based position and velocity controller (identical to Architecture 2)
- Inner loop: INDI-based attitude and angular rate controller for improved disturbance rejection and robustness
- Actuator model and sensor data used directly in the INDI control law

---

## Prerequisites

- **MATLAB / Simulink** (R2024a or later recommended)
- **ArduPilot SITL** — [Setup guide](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html)
- **Python 3.10+** with `pymavlink`, `dronekit` (for MAVLink scripting)
- **QGroundControl** or **Mission Planner** (optional, for GCS)
- Linux recommended; (Not tested for Windows or MacOS as yet)

---

## Getting Started

1. Clone the repository:

```bash
git clone https://github.com/<your-org>/<repo-name>.git
cd <repo-name>
```

2. Launch a simulation — see [Usage](#usage) below.

---

## Directory Structure

```
.
├── MATLAB/
│   ├── Controllers/                # controller dev folder
│   ├── Copter/                # Hexsoon vehicle parameters and airframe config
│       ├── SIM_multicopter/                # Run for full ardupilot SITL
│       ├── SIM_multicopter_PID_noZaccel/                # Run for full MATLAB SITL (No Alt Accel inner loop PID)
│       ├── SIM_multicopter_PID_Zaccel/                # Run for full MATLAB SITL with Alt Accel inner loop PID (Uses Ardupilot architecture and gains)
│       ├── init_hexsoon.m/                # init file which generates Hexsoon.mat config file 
│       ├── Hexsoon.mat/                # MAT file with Hexsoon copter config
│       ├── readme.md/               
│   ├── bprl_copter/                # BPRL copter parameters and airframe config
│   ├── rtos_drone/                # BPRL experimental RTOS copter parameters and airframe config
│   ├── tcp_udp_ip_2.0.6/                
│   ├── SITL_connector.m      
│   ├── readme.m                
└── python (TBD)
│   
├── visualizer/
│   ├── quad_viz_pybullet/                # pybullet matlab visualizer

├── .gitmodules                     # Git submodule config (ArduPilot)
└── README.md
```

---

## Usage

1. Open MATLAB and navigate to the relevant vehicle folder. 

### ArduPilot Only (Baseline)

See readme_ardupilot_only.md

### Full MATLAB PID Controller

1. Run the initialisation script for the target platform:

```matlab
init_hexsoon   % or init_<vehicle_name>
```

2. Run visualizer script

```python
python drone_viz.py 
```


2. Open and run the MATLAB script

```matlab
SIM_multicopter_PID_noZaccel.m
```

### PID + INDI Controller

1. Run the initialisation script for the target platform:

```matlab
init_hexsoon   % or init_<vehicle_name>
```

3. Open and run the MATLAB script

```matlab
SIM_multicopter_PID_INDI.m
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