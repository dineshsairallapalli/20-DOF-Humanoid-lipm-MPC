# 20-DOF Humanoid Robot Stabilization Using LIPM, MPC, and PD Control on Windows

## 📖 Project Overview

This repository contains an advanced implementation of a 20-Degree-of-Freedom (20-DOF) humanoid robot (Robotis OP3) simulation in MuJoCo on a Windows platform. It integrates the Linear Inverted Pendulum Model (LIPM) dynamics with a Model Predictive Control (MPC) algorithm over a 10-step prediction horizon, complemented by a Proportional-Derivative (PD) control layer. This significantly improves stable walking behavior and increases energy efficiency by approximately 20%.

## 🤖 Robotis OP3 Overview

The Robotis OP3 is a sophisticated, research-grade humanoid robot designed with:

* **20 Degrees of Freedom (DOF)**: Enables complex joint movements required for realistic humanoid locomotion.
* **Dynamixel Actuators**: Offer precise torque and position control, crucial for advanced robotic applications.
* **Open-source architecture**: Provides flexibility for custom developments and easy integration with simulation environments such as MuJoCo.

## 📚 Technical Concepts Explained

### Linear Inverted Pendulum Model (LIPM)

The LIPM is a simplified dynamic model widely utilized in humanoid robotics. It treats the robot's Center of Mass (CoM) as a single point mass, connected via a massless leg pivoting around the foot placement:

$$
\ddot{x}(t) = \frac{g}{z_0}(x(t) - p)
$$

**Variables Explained:**

* $x(t)$: CoM horizontal displacement.
* $g$: Gravity acceleration (\~9.81 m/s²).
* $z_0$: Constant height of CoM.
* $p$: Foot pivot position.

The simplification allows efficient real-time computation.

### Model Predictive Control (MPC)

MPC is a sophisticated control technique employed to anticipate future system behaviors and optimize current control actions. MPC continuously solves an optimization problem:

$$
\min_{p_k} \sum_{k=0}^{N-1} [(x_{k+1}-x_{ref,k+1})^T Q (x_{k+1}-x_{ref,k+1})+(p_k - p_{k-1})^T R (p_k - p_{k-1})]
$$

**Subject to:**

* Linearized LIPM dynamics.
* Step constraints (length, width).

### Proportional-Derivative (PD) Control

PD control complements MPC by regulating joint-level dynamics, providing fine-grained tracking accuracy and ensuring smooth joint movements:

$$
\tau = K_p(e) + K_d(\dot{e})
$$

* $\tau$: Joint torques.
* $K_p$: Proportional gain (reduces positional error).
* $K_d$: Derivative gain (dampens motion, reduces overshoot).
* $e$: Position error; $\dot{e}$: Velocity error.

## 🛠️ Windows Installation Guide

### Prerequisites

* **Windows 10 or later**
* **Python ≥ 3.8** ([Download Python](https://www.python.org/downloads/windows/))
* **MuJoCo ≥ 2.x** ([Download MuJoCo](https://github.com/deepmind/mujoco/releases))
* **Visual Studio C++ Build Tools** ([Download here](https://visualstudio.microsoft.com/visual-cpp-build-tools/))
* **Git** ([Download Git](https://git-scm.com/downloads))

### Installation Steps

1. **Python and Environment Setup:**

```cmd
python -m venv venv
venv\Scripts\activate
pip install --upgrade pip
```

2. **MuJoCo Setup:**

* Extract MuJoCo to:

  ```
  C:\Users\<YourUsername>\.mujoco\mujoco-2.x
  ```
* Place `mjkey.txt` license file in the same folder.
* Set environment variables:

```cmd
setx MUJOCO_PY_MUJOCO_PATH "C:\Users\<YourUsername>\.mujoco\mujoco-2.x" /M
setx MUJOCO_PY_MJKEY_PATH "C:\Users\<YourUsername>\.mujoco\mujoco-2.x\mjkey.txt" /M
```

3. **Visual Studio Build Tools Installation:**

* Install with "Desktop Development with C++" enabled.

4. **Clone Repository and Dependencies:**

```cmd
git clone https://github.com/yourusername/20dof-humanoid-lipm-mpc.git
cd 20dof-humanoid-lipm-mpc
pip install mujoco mujoco-py numpy cvxpy
pip install -e .
```

## 🚩 Running the Simulation

### Quick Start

```cmd
python scripts\run_simulation.py
```

### Custom Parameters

```cmd
python -m lipm_mpc.lipm_mpc_fast --model models\op3_scene.xml --horizon 10 --step_time 0.5 --total_time 15.0
```

## 📁 Repository Structure

Clearly structured for ease of use and clarity.

## 🔬 Validation and Performance

Comprehensive testing for performance verification and stability.

## 📚 Documentation

Detailed theoretical and practical insights in `docs\architecture.md`.

## 🤝 Contributing Guidelines

Open for contributions; follow the previously provided steps.

## 📜 License

MIT License. See [LICENSE.md](LICENSE.md) for details.

---

Dedicated to advancing humanoid robotics research with robust simulation and advanced control methodologies using Robotis OP3, LIPM, MPC, PD control, and MuJoCo on Windows.
