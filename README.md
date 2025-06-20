# 20-DOF-Humanoid-lipm-MPC
MuJoCo based 20-DOF humanoid walking with LIPM and 10-step horizon MPC +20% efficiency
# 20-DOF Humanoid LIPM-MPC

[![Build Status](https://github.com/yourusername/20dof-humanoid-lipm-mpc/actions/workflows/ci.yml/badge.svg)](https://github.com/yourusername/20dof-humanoid-lipm-mpc/actions) [![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE.md)

## 📖 Overview

This repository implements a **20-DOF humanoid walking simulation** in MuJoCo with:

* **Linear Inverted Pendulum Model (LIPM)** dynamics
* **10-step horizon Model Predictive Control (MPC)**
* **20% efficiency gain** over baseline walking

Developed for an M.S. thesis, it covers model creation, control design, simulation, and performance analysis.

## 📚 Background Concepts

### Linear Inverted Pendulum Model (LIPM)

LIPM approximates a biped as a point mass at a fixed height $z_0$ above the ground, attached to the stance foot by a massless leg. The horizontal CoM dynamics become linear:

```text
x_ddot = (g / z0) * (x - p)
```

* **x**: horizontal CoM position
* **p**: stance foot position
* **g**: gravitational acceleration
* **z0**: CoM height

Analytic solution over time $t$:

```text
x(t) = x0 * cosh(t / tau) + tau * xdot0 * sinh(t / tau)
tau = sqrt(z0 / g)
```

where **x0**, **xdot0** are initial CoM state and $\tau$ is the pendulum time constant.

### Model Predictive Control (MPC)

MPC solves for an optimal sequence of foot placements $p_{0..N-1}$ over a finite horizon $N$ by minimizing tracking error and step variation:

```text
minimize sum_{k=0}^{N-1} [ (x_{k+1} - x_ref_{k+1})^2 + R * (p_k - p_{k-1})^2 ]
```

subject to:

* Discrete LIPM dynamics (derived from the continuous model)
* Constraints on step length/width

After each solve, only the first placement $p_0$ is executed, then the horizon shifts forward (receding horizon).

---

## 🎯 Key Features

* **Modular design**: clear separation of model, control, and utilities
* **Package-ready**: installable via `setup.py`, importable as `lipm_mpc`
* **Examples**: simple and advanced scripts in `examples/`
* **CI & testing**: automated via GitHub Actions and pytest
* **Extensible**: swap models or tune MPC parameters easily

## 📑 Table of Contents

1. [Installation](#installation)
2. [Configuration](#configuration)
3. [Usage](#usage)
4. [Examples](#examples)
5. [Architecture](#architecture)
6. [Directory Structure](#directory-structure)
7. [Testing & CI](#testing--ci)
8. [Contributing](#contributing)
9. [License](#license)

---

## ⚙️ Installation

### Prerequisites

* **Python 3.8+**, **Git**, **MuJoCo 2.x** with valid license
* System libs: OpenGL, GLFW

### Setup Steps

```bash
git clone https://github.com/yourusername/20dof-humanoid-lipm-mpc.git
cd 20dof-humanoid-lipm-mpc
python3 -m venv venv
source venv/bin/activate  # macOS/Linux
# or venv\Scripts\activate  # Windows
pip install --upgrade pip
pip install -r requirements.txt
pip install -e .
```

Configure MuJoCo license:

```bash
export MUJOCO_PY_MJKEY_PATH=/path/to/mjkey.txt
export MUJOCO_PY_MUJOCO_PATH=/path/to/mujoco-2.x
```

---

## 🔧 Configuration

By default, uses `models/scene.xml`. To override:

```python
from lipm_mpc.lipm_mpc_fast import run_simulation
run_simulation(
  model_path='path/to/your_model.xml',
  horizon_steps=10,
  step_time=0.5,
  total_time=15.0
)
```

| Parameter       | Description                    | Default            |
| --------------- | ------------------------------ | ------------------ |
| `model_path`    | MuJoCo XML file                | `models/scene.xml` |
| `horizon_steps` | MPC horizon steps              | `10`               |
| `step_time`     | Single-step duration (seconds) | `0.5`              |
| `total_time`    | Total simulation time (sec)    | `15.0`             |

---

## 🚀 Usage

### Quick Start

```bash
./scripts/run_simulation.sh
# or:
python -m lipm_mpc.lipm_mpc_fast --model models/scene.xml
```

### CLI Options

```text
Usage: python -m lipm_mpc.lipm_mpc_fast [--model PATH] [--horizon N] [--step_time T] [--total_time T]
```

* `--model`: path to XML model
* `--horizon`: MPC horizon length
* `--step_time`: step time in seconds
* `--total_time`: simulation duration

---

## 📂 Examples

* **Minimal**: `examples/simple_walk.py`
* **Benchmarking**: `examples/benchmark.py`

---

## 🏗️ Architecture

Detailed in `docs/architecture.md`. Briefly:

1. **Model Layer**: `models/scene.xml`
2. **Control Layer**: `lipm_mpc/lipm_mpc_fast.py`
3. **Visualization**: via `mujoco-viewer`

---

## 📁 Directory Structure

```text
20dof-humanoid-lipm-mpc/
├── .github/
│   └── workflows/ci.yml
├── docs/
│   ├── index.md
│   └── architecture.md
├── examples/
├── scripts/
├── lipm_mpc/
├── models/
├── tests/
├── requirements.txt
├── setup.py
├── README.md
├── LICENSE.md
└── .gitignore
```

---

## ✅ Testing & CI

```bash
pytest --disable-warnings -q
```

Automated via GitHub Actions (`.github/workflows/ci.yml`).

---

## 🤝 Contributing

1. Fork & clone
2. `git checkout -b feature/xyz`
3. Commit & push
4. Open PR

See \[CODE\_OF\_CONDUCT.md].

---

## 📜 License

MIT License — see `LICENSE.md`.

*Crafted for advanced bipedal robotics research*
