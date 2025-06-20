# 20-DOF Humanoid Robot Stabilization Using LIPM and MPC on Windows

## 📖 Project Overview

This repository contains an advanced implementation of a 20-Degree-of-Freedom (20-DOF) humanoid robot (Robotis OP3) simulation in MuJoCo on a Windows platform. It integrates the Linear Inverted Pendulum Model (LIPM) dynamics with a Model Predictive Control (MPC) algorithm over a 10-step prediction horizon, significantly improving stable walking behavior and increasing energy efficiency by approximately 20%.

## 🤖 Robotis OP3 Overview

The Robotis OP3 is a sophisticated, research-grade humanoid robot designed with:

* **20 Degrees of Freedom (DOF)**: Allows for complex articulation of joints necessary for realistic humanoid motion.
* **Dynamixel Actuators**: High precision and torque-controllable actuators, ideal for precise and robust robotic applications.
* **Open-source architecture**: Enables extensive customization and robust integration with simulation platforms like MuJoCo.

## 📚 Technical Concepts Explained

### Linear Inverted Pendulum Model (LIPM)

The LIPM is a simplified yet powerful dynamic model commonly employed in humanoid robotics. It models the robot’s Center of Mass (CoM) as a single point mass attached via a massless rod (leg) that pivots around the foot placement position. This approximation allows simplified linear dynamics to accurately describe CoM behavior:

$$
\ddot{x}(t) = \frac{g}{z_0}(x(t) - p)
$$

**Where:**

* $x(t)$: Horizontal displacement of the CoM.
* $g$: Acceleration due to gravity (\~9.81 m/s²).
* $z_0$: Constant vertical height of the CoM above the ground.
* $p$: Position of the foot pivot point on the ground.

This simplification allows for computational efficiency, making real-time control feasible and effective.

### Model Predictive Control (MPC)

MPC is an advanced control strategy widely used in robotics for managing systems with constraints and predictive decision-making requirements. MPC solves an optimization problem over a finite future horizon, aiming to find the optimal sequence of control actions (foot placements for walking robots) by minimizing a cost function:

$$
\min_{p_k} \sum_{k=0}^{N-1} \left[(x_{k+1} - x_{ref,k+1})^T Q (x_{k+1} - x_{ref,k+1}) + (p_k - p_{k-1})^T R (p_k - p_{k-1})\right]
$$

Subject to the following constraints:

* Linearized discrete-time dynamics of the LIPM.
* Physical constraints on foot placement, such as maximum step length and width.

Here,

* $Q$ and $R$ are weighting matrices balancing the tracking accuracy of the reference trajectory $x_{ref}$ and the smoothness of control actions.
* $N$ represents the prediction horizon length.

## 🛠️ Windows Installation Guide

### Prerequisites

* **Windows 10 or later**
* **Python ≥ 3.8** ([Download Python](https://www.python.org/downloads/windows/))
* **MuJoCo ≥ 2.x** ([Download MuJoCo](https://github.com/deepmind/mujoco/releases))
* **Visual Studio C++ Build Tools** ([Download here](https://visualstudio.microsoft.com/visual-cpp-build-tools/))
* **Git** ([Download Git](https://git-scm.com/downloads))

### Installation Steps

1. **Install Python and create a virtual environment:**

* Install Python and ensure it’s added to your PATH during installation.
* Open Command Prompt:

```cmd
python -m venv venv
venv\Scripts\activate
pip install --upgrade pip
```

2. **Install MuJoCo:**

* Extract the downloaded MuJoCo archive to:

  ```
  C:\Users\<YourUsername>\.mujoco\mujoco-2.x
  ```

* Obtain and place your MuJoCo license key (`mjkey.txt`) in the directory:

  ```
  C:\Users\<YourUsername>\.mujoco\mujoco-2.x\mjkey.txt
  ```

* Set MuJoCo environment variables (open a new Command Prompt as administrator):

```cmd
setx MUJOCO_PY_MUJOCO_PATH "C:\Users\<YourUsername>\.mujoco\mujoco-2.x" /M
setx MUJOCO_PY_MJKEY_PATH "C:\Users\<YourUsername>\.mujoco\mujoco-2.x\mjkey.txt" /M
```

**Restart Command Prompt after this step.**

3. **Install Visual Studio Build Tools:**

* Run the installer downloaded earlier.
* Select "Desktop Development with C++" and complete the installation.

4. **Clone the Repository and Install Dependencies:**

Open Command Prompt:

```cmd
git clone https://github.com/dineshsairallapalli/20dof-humanoid-lipm-mpc.git
cd 20dof-humanoid-lipm-mpc
pip install mujoco mujoco-py numpy cvxpy
pip install -e .
```

## 🚩 Simulation Execution

### Quick Start

In Command Prompt, execute:

```cmd
python scripts\run_simulation.py
```

### Custom Run Parameters

To customize parameters such as horizon, step time, and simulation duration, use:

```cmd
python -m lipm_mpc.lipm_mpc_fast --model models\op3_scene.xml --horizon 10 --step_time 0.5 --total_time 15.0
```
---

Dedicated to advancing humanoid robotics research through detailed simulation and control methodology using Robotis OP3, LIPM, MPC, and MuJoCo on Windows.
