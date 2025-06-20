import math
import numpy as np
import cvxpy as cp
import mujoco
import mujoco.viewer

# ------------------------------
# Constants and LIPM parameters
# ------------------------------
g = 9.81                # gravitational acceleration (m/s^2)
h = 0.3                 # approximate CoM height (m)
omega = math.sqrt(g/h)  # natural frequency

# Step and simulation parameters
step_length = 0.05      # nominal step length (m)
step_time = 0.6         # reduced step duration (sec) for faster stepping
dt = 0.005              # simulation time step (sec)

# --- Controller Gains ---
# For swing leg ankle PD control
Kp_ankle = 100.0
Kd_ankle = 10.0
ankle_gain = 0.01

# For swing leg knee PD control
knee_amp = 0.2          # maximum knee flexion (rad)
Kp_knee = 50.0          # knee proportional gain
Kd_knee = 5.0           # knee derivative gain
knee_scaling = 0.5      # extra contribution from ankle adjustment

# For support leg hip control (to lean forward)cm
Kp_hip = 50.0
Kd_hip = 5.0
desired_hip_angle = 0.1  # desired hip pitch (rad)

# Debug parameters
debug = True
debug_interval = 0.1    # sec between debug prints

# ------------------------------
# LIPM Trajectory Generator
# ------------------------------
def lipm_trajectory(x0, x_dot0, p, t):
    """
    Computes desired CoM x-position and velocity at time t
    using the analytical solution of the linear inverted pendulum model.
    """
    A = x0 - p
    B = x_dot0 / omega
    x_t = A * math.cosh(omega * t) + B * math.sinh(omega * t) + p
    x_dot_t = A * omega * math.sinh(omega * t) + B * omega * math.cosh(omega * t)
    return x_t, x_dot_t

# ------------------------------
# MPC Solver for Foot Placement
# ------------------------------
def solve_mpc(x0, x_dot0, step_length, horizon=10):
    T = step_time
    A_step = np.array([[math.cosh(omega * T), math.sinh(omega * T)/omega],
                       [omega * math.sinh(omega * T), math.cosh(omega * T)]])
    B_step = np.array([[1 - math.cosh(omega * T)],
                       [-omega * math.sinh(omega * T)]])
    
    N = horizon
    p = cp.Variable(N)
    x_vars = [cp.Variable(2) for _ in range(N+1)]
    constraints = [x_vars[0] == np.array([x0, x_dot0])]
    for i in range(N):
        constraints.append(x_vars[i+1] == A_step @ x_vars[i] + B_step.flatten() * p[i])
    # Nominal foot placement: step_length added to x0.
    p_nominal = x0 + step_length
    x_ref = np.array([p_nominal, 0.0])
    cost = sum(cp.square(p[i] - p_nominal) for i in range(N))
    Qf = np.diag([10, 1])
    cost += cp.quad_form(x_vars[N] - x_ref, Qf)
    prob = cp.Problem(cp.Minimize(cost), constraints)
    prob.solve(solver=cp.OSQP)
    if prob.status != cp.OPTIMAL:
        print("Warning: MPC optimization did not solve optimally.")
    return p.value[0]

# ------------------------------
# Main Simulation Loop
# ------------------------------
def main():
    # Load model and data.
    model = mujoco.MjModel.from_xml_path("scene.xml")
    data = mujoco.MjData(model)
    
    # Retrieve key IDs.
    torso_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "torso")
    
    # Ankle joints and actuators.
    l_ank_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "l_ank_pitch")
    r_ank_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "r_ank_pitch")
    l_ank_act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "l_ank_pitch_act")
    r_ank_act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "r_ank_pitch_act")
    
    # Knee joints and actuators.
    l_knee_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "l_knee")
    r_knee_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "r_knee")
    l_knee_act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "l_knee_act")
    r_knee_act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "r_knee_act")
    
    # Hip actuators.
    l_hip_act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "l_hip_pitch_act")
    r_hip_act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "r_hip_pitch_act")
    
    # Initialize CoM state (x-direction).
    x0 = 0.0
    x_dot0 = 0.0
    
    # Start with left leg as support.
    left_support = True
    t_total = 0.0
    next_debug_time = 0.0
    
    with mujoco.viewer.launch_passive(model, data) as v:
        while v.is_running():
            # At the start of a step, compute optimal foot placement.
            if t_total < 1e-6:
                optimal_p = solve_mpc(x0, x_dot0, step_length, horizon=3)
            
            # Get current CoM state.
            com_pos = data.site_xpos[torso_site_id, 0]
            vel = np.zeros((6, 1), dtype=np.float64)
            mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_SITE, torso_site_id, vel, 0)
            com_vel = vel[0, 0]
            
            # Compute desired CoM state from LIPM.
            desired_com, desired_vel = lipm_trajectory(x0, x_dot0, optimal_p, t_total)
            error_pos = desired_com - com_pos
            error_vel = desired_vel - com_vel
            
            # Compute ankle adjustment command.
            desired_acc = Kp_ankle * error_pos + Kd_ankle * error_vel
            ankle_adjustment = desired_acc * ankle_gain
            
            # --- Assign commands based on support leg ---
            if left_support:
                # Left leg is support: its ankle stays fixed.
                support_ankle_cmd = 0.0
                # Right (swing) ankle gets the adjustment.
                swing_ankle_cmd = data.qpos[r_ank_joint_id] + ankle_adjustment
                data.ctrl[l_ank_act_id] = support_ankle_cmd
                data.ctrl[r_ank_act_id] = swing_ankle_cmd
                
                # Knee commands: support knee is kept near zero;
                # swing knee follows a slow sine trajectory plus adjustment.
                support_knee_des = 0.0
                swing_sine = knee_amp * math.sin(math.pi * t_total / step_time)
                swing_knee_des = swing_sine + knee_scaling * ankle_adjustment
                left_knee_cmd = Kp_knee * (support_knee_des - data.qpos[l_knee_joint_id]) - Kd_knee * data.qvel[l_knee_joint_id]
                right_knee_cmd = Kp_knee * (swing_knee_des - data.qpos[r_knee_joint_id]) - Kd_knee * data.qvel[r_knee_joint_id]
                data.ctrl[l_knee_act_id] = left_knee_cmd
                data.ctrl[r_knee_act_id] = right_knee_cmd
                
                # Hip control on support leg (left) to lean forward.
                left_hip_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "l_hip_pitch")
                hip_cmd = Kp_hip * (desired_hip_angle - data.qpos[left_hip_joint_id]) - Kd_hip * data.qvel[left_hip_joint_id]
                data.ctrl[l_hip_act_id] = hip_cmd
                # Swing hip remains at zero.
                data.ctrl[r_hip_act_id] = 0.0
                
            else:
                # Right leg is support.
                support_ankle_cmd = 0.0
                swing_ankle_cmd = data.qpos[l_ank_joint_id] + ankle_adjustment
                data.ctrl[r_ank_act_id] = support_ankle_cmd
                data.ctrl[l_ank_act_id] = swing_ankle_cmd
                
                support_knee_des = 0.0
                swing_sine = knee_amp * math.sin(math.pi * t_total / step_time)
                swing_knee_des = swing_sine + knee_scaling * ankle_adjustment
                right_knee_cmd = Kp_knee * (support_knee_des - data.qpos[r_knee_joint_id]) - Kd_knee * data.qvel[r_knee_joint_id]
                left_knee_cmd = Kp_knee * (swing_knee_des - data.qpos[l_knee_joint_id]) - Kd_knee * data.qvel[l_knee_joint_id]
                data.ctrl[r_knee_act_id] = right_knee_cmd
                data.ctrl[l_knee_act_id] = left_knee_cmd
                
                right_hip_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "r_hip_pitch")
                hip_cmd = Kp_hip * (desired_hip_angle - data.qpos[right_hip_joint_id]) - Kd_hip * data.qvel[right_hip_joint_id]
                data.ctrl[r_hip_act_id] = hip_cmd
                data.ctrl[l_hip_act_id] = 0.0
                
            # --- Debug output ---
            if debug and t_total >= next_debug_time:
                if left_support:
                    print(f"Step Time: {t_total:6.3f} sec, Support: left")
                    print(f"  CoM x: measured {com_pos:7.4f}, desired {desired_com:7.4f}, error {error_pos:7.4f}")
                    print(f"  Support (left) ankle: {support_ankle_cmd:7.4f}, Swing (right) ankle: {swing_ankle_cmd:7.4f}")
                    print(f"  l_knee (support): cmd {left_knee_cmd:7.4f}, r_knee (swing): cmd {right_knee_cmd:7.4f}")
                    left_hip_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "l_hip_pitch")
                    print(f"  l_hip qpos: {data.qpos[left_hip_joint_id]:7.4f}, cmd {hip_cmd:7.4f}")
                else:
                    print(f"Step Time: {t_total:6.3f} sec, Support: right")
                    print(f"  CoM x: measured {com_pos:7.4f}, desired {desired_com:7.4f}, error {error_pos:7.4f}")
                    print(f"  Support (right) ankle: {support_ankle_cmd:7.4f}, Swing (left) ankle: {swing_ankle_cmd:7.4f}")
                    right_hip_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "r_hip_pitch")
                    print(f"  r_hip qpos: {data.qpos[right_hip_joint_id]:7.4f}, cmd {hip_cmd:7.4f}")
                print("-" * 60)
                next_debug_time += debug_interval
            
            # Step the simulation and sync the viewer.
            mujoco.mj_step(model, data)
            v.sync()
            t_total += dt
            
            # End of step: update CoM state and swap support leg.
            if t_total >= step_time:
                x0 = data.site_xpos[torso_site_id, 0]
                vel = np.zeros((6, 1), dtype=np.float64)
                mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_SITE, torso_site_id, vel, 0)
                x_dot0 = vel[0, 0]
                t_total = 0.0
                next_debug_time = 0.0
                left_support = not left_support

if __name__ == "__main__":
    main()
