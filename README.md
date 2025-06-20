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

## 🖥️ Code and Control Implementation

### Core Controller Concepts

The MPC controller implemented in `lipm_mpc_fast.py` uses the linearized discrete-time version of LIPM dynamics. It solves the optimization problem iteratively at each simulation timestep, providing a dynamically optimal foot placement to maintain stability.

### Key Functions and Files:

* `lipm_mpc_fast.py`: Core MPC logic, implementing the optimization and updating control actions.
* `op3_scene.xml`: MuJoCo model of Robotis OP3, including physical properties and initial configurations.

### Tuning the MPC Controller

Controller tuning involves adjusting key parameters:

* **Prediction Horizon (`N`)**: Determines how far into the future the MPC predicts. A longer horizon provides better stability at computational cost.
* **Weight Matrices (`Q` and `R`)**: Balances tracking accuracy and smoothness of control actions. Typically tuned via trial and error or systematic approaches.
* **Step Size (`step_time`)**: Affects responsiveness and stability. Shorter steps make the robot responsive but may introduce instability if too short.

Adjust these parameters in the script or through command-line arguments to achieve the desired performance:

```cmd
python -m lipm_mpc.lipm_mpc_fast --model models\op3_scene.xml --horizon 10 --step_time 0.5 --total_time 15.0
```

## 🛠️ Windows Installation Guide

Please follow the previously outlined installation steps carefully for a successful setup.

## 🚩 Simulation Execution

### Quick Start

```cmd
python scripts\run_simulation.py
```

## 📁 Repository Structure

The structure is clearly outlined above for easy navigation and understanding.

## 🔬 Validation and Performance

* Tests in `tests\` confirm the controller’s performance and stability.
* Performance metrics and validation procedures are detailed in the documentation.

## 📚 Detailed Documentation

For further details, theoretical insights, and implementation specifics, refer to the comprehensive documentation in `docs\architecture.md`.

## 🤝 Contributing Guidelines

Contributions are highly welcomed:

* Fork the repository.
* Create a feature branch (`git checkout -b feature-name`).
* Commit your changes (`git commit -m "feature details"`).
* Push your branch (`git push origin feature-name`).
* Open a Pull Request.

## 📜 License

This project is licensed under the MIT License. See [LICENSE.md](LICENSE.md) for full details.

---

Dedicated to advancing humanoid robotics research through detailed simulation and control methodology using Robotis OP3, LIPM, MPC, and MuJoCo on Windows.
