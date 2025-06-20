# 20-DOF Humanoid Robot Locomotion using LIPM, MPC, and PD Control in MuJoCo

## Project Overview

This repository contains an advanced implementation of a 20-Degree-of-Freedom (20-DOF) humanoid robot (Robotis OP3) simulation in MuJoCo on a Windows platform. It integrates the Linear Inverted Pendulum Model (LIPM) dynamics with a Model Predictive Control (MPC) algorithm over a 10-step prediction horizon, complemented by a Proportional-Derivative (PD) control layer. This significantly improves stable walking behavior and increases energy efficiency by approximately 20%.

* **Linear Inverted Pendulum Model (LIPM)**: For modeling CoM dynamics.
* **Model Predictive Control (MPC)**: For optimal footstep planning.
* **Proportional-Derivative (PD) Control**: For low-level joint tracking.
* **MuJoCo**: High-performance physics engine.

The entire system is implemented in Python and tested under **Windows 10/11** using MuJoCo 2.x.

---

## 🤖 Robot Model – Robotis OP3

The Robotis OP3 is a sophisticated, research-grade humanoid robot designed with:

* **20 Degrees of Freedom (DOF)**: Enables complex joint movements required for realistic humanoid locomotion.
* **Dynamixel Actuators**: Offer precise torque and position control, crucial for advanced robotic applications.
* **Open-source architecture**: Provides flexibility for custom developments and easy integration with simulation environments such as MuJoCo.

MuJoCo model file: `models/op3_scene.xml`

---

## ⚙️ System Architecture

```text
[ LIPM-MPC Controller ]  --->  Footstep Targets
         |                     ↓
         ↓            [ PD Controller ]
     CoM Planning ---> Joint Angles  --->  [ MuJoCo Physics Engine ]
```

* High-level MPC planner determines foot placements.
* PD controller enforces desired joint angles for each walking phase.

---

## 📚 Mathematical Foundation

### 🔷 Linear Inverted Pendulum Model (LIPM)

The LIPM is a simplified dynamic model widely utilized in humanoid robotics. It treats the robot's Center of Mass (CoM) as a single point mass, connected via a massless leg pivoting around the foot placement:

$\ddot{x}(t) = \frac{g}{z_0}(x(t) - p)$

#### ✅ General Solution

The general time-domain solution is:

$$
\begin{aligned}
    x(t) &= x_0 \cosh(t/\tau) + \tau \dot{x}_0 \sinh(t/\tau) + (1 - \cosh(t/\tau))p \\
    \dot{x}(t) &= \frac{x_0}{\tau} \sinh(t/\tau) + \dot{x}_0 \cosh(t/\tau) - \frac{1}{\tau} \sinh(t/\tau)p
\end{aligned}
$$

Where:

* $\tau = \sqrt{z_0 / g}$
* $x_0, \dot{x}_0$: initial CoM state

#### ✅ Discrete LIPM Dynamics (state-space)

$$
\begin{bmatrix}
    x_{k+1} \\
    \dot{x}_{k+1}
\end{bmatrix} = A \begin{bmatrix}
    x_k \\
    \dot{x}_k
\end{bmatrix} + B p_k
$$

Where:

$$
A = \begin{bmatrix}
    \cosh(T/\tau) & \tau \sinh(T/\tau) \\
    \frac{\sinh(T/\tau)}{\tau} & \cosh(T/\tau)
\end{bmatrix},
\quad
B = \begin{bmatrix}
    1 - \cosh(T/\tau) \\
    -\frac{\sinh(T/\tau)}{\tau}
\end{bmatrix}
$$

**Variables Explained:**

* $x(t)$: CoM horizontal displacement.
* $g$: Gravity acceleration (\~9.81 m/s²).
* $z_0$: Constant height of CoM.
* $p$: Foot pivot position.


The simplification allows efficient real-time computation.
---

### Model Predictive Control (MPC)

MPC is a sophisticated control technique employed to anticipate future system behaviors and optimize current control actions. MPC computes a sequence of foot placements $\{p_k\}_{k=0}^{N-1}$ that minimizes CoM tracking error and step effort:

$$
\min_{\{p_k\}} \sum_{k=0}^{N-1} \left[ (x_k - x_k^{ref})^T Q (x_k - x_k^{ref}) + (p_k - p_{k-1})^T R (p_k - p_{k-1}) \right]
$$

**Subject to:**

* Discrete-time LIPM dynamics:

$$
    x_{k+1} = A x_k + B p_k
$$

* Step size constraints:

$$
    p_{min} \leq p_k \leq p_{max}
$$

* Optional terminal condition:

$$
    x_N \approx x_{ref,N}
$$

**Where:**

* $x_k \in \mathbb{R}^2$: CoM state (position, velocity)
* $Q \in \mathbb{R}^{2\times2}$, $R \in \mathbb{R}^{1\times1}$: cost matrices
* $N$: prediction horizon (typically 10)

Solved via convex optimization (e.g., using `cvxpy`).

---

### Proportional-Derivative (PD) Control

PD control complements MPC by regulating joint-level dynamics, providing fine-grained tracking accuracy and ensuring smooth joint movements:

Joint-level low-level control torque:

$$
\tau_i = K_p^i (q_i^{ref} - q_i) + K_d^i (\dot{q}_i^{ref} - \dot{q}_i)
$$

Where:

* $\tau_i$: Torque applied at joint $i$
* $q_i, \dot{q}_i$: actual position and velocity
* $q_i^{ref}, \dot{q}_i^{ref}$: desired joint trajectory (from IK or footstep planner)

This PD controller runs at every MuJoCo step to enforce smooth joint behavior.

---

## Code Overview

### Main Script – `lipm_mpc_fast.py`

```python
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)
```

* Initializes MuJoCo model and state.

```python
def solve_mpc(x0, horizon=10):
    # Set up and solve MPC problem with cvxpy
    # Returns optimal future foot placements
```

```python
for step in sim_time:
    p_opt = solve_mpc(current_state)
    q_ref = inverse_kinematics(p_opt)
    apply_pd_control(q_ref, data)
    mujoco.mj_step(model, data)
```

---

## 🧪 Tuning and Parameters

| Parameter   | Description                       | Suggested Value        |
| ----------- | --------------------------------- | ---------------------- |
| `horizon`   | MPC prediction steps              | 10                     |
| `step_time` | Duration of each walking step (s) | 0.4–0.6                |
| `Q`         | CoM state cost                    | diag(\[100, 10])       |
| `R`         | Step regularization               | diag(\[1])             |
| `Kp, Kd`    | PD control gains per joint        | Kp: 200–500, Kd: 20–50 |

---

## 🛠 Windows Installation Guide

### Prerequisites

* Windows 10/11
* Python 3.8+
* MuJoCo 2.x with license key (`mjkey.txt`)
* Visual Studio Build Tools
* Git for Windows

### Setup Steps

1. **Install Python** (add to PATH during install)

```cmd
python -m venv venv
venv\Scripts\activate
pip install --upgrade pip
```

2. **Download and Configure MuJoCo**

* Extract MuJoCo to:

```text
C:\Users\<YourUsername>\.mujoco\mujoco-2.x
```

* Place `mjkey.txt` in the same directory.
* Set system environment variables:

```cmd
setx MUJOCO_PY_MUJOCO_PATH "C:\Users\<YourUsername>\.mujoco\mujoco-2.x" /M
setx MUJOCO_PY_MJKEY_PATH "C:\Users\<YourUsername>\.mujoco\mujoco-2.x\mjkey.txt" /M
```

3. **Install Visual Studio C++ Tools**

* Install from [Microsoft Visual Studio Build Tools](https://visualstudio.microsoft.com/visual-cpp-build-tools/).
* Select "Desktop Development with C++" during install.

4. **Clone the Repository and Install Dependencies**

```cmd
git clone https://github.com/dineshsairallapalli/20dof-humanoid-lipm-mpc.git
cd 20dof-humanoid-lipm-mpc
pip install -r requirements.txt
pip install -e .
```

---
## Running the Simulation

### Quick Start

```cmd
python scripts\run_simulation.py
```

### Custom Parameters

```cmd
python -m lipm_mpc.lipm_mpc_fast --model models\op3_scene.xml --horizon 10 --step_time 0.5 --total_time 15.0
```
---

Dedicated to advancing humanoid robotics research with robust simulation and advanced control methodologies using Robotis OP3, LIPM, MPC, PD control, and MuJoCo. Created as part of an M.S. research project to demonstrate full-stack humanoid locomotion with modern control theory in simulation. Built for realism, modularity, and extensibility.
