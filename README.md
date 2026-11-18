# Biorobotics: Legged Locomotion, Flapping-Wing Control and Reinforcement Learning

Quadruped gait generation and inverse kinematics in **ROS 2 + Gazebo**, attitude control of a
flapping-wing robot in **Simulink**, and **reinforcement learning** agents (Q-learning, DQN, PPO,
DDPG) in MATLAB. Course: *Biorobotics* (R7017E), Luleå University of Technology, 2026.

**Stack:** Python · ROS 2 (rclpy, actions) · Gazebo · NumPy · MATLAB · Simulink · Reinforcement Learning Toolbox

## Projects

| Folder | Topic | What we built |
|---|---|---|
| [`lab1/`](lab1) | Quadruped gait | Keyframe-based walking gait: foot-position keyframes sent to a simulated quadruped in Gazebo (`task.py`) |
| [`lab2-quadruped-inverse-kinematics/`](lab2-quadruped-inverse-kinematics) | Inverse kinematics | Analytical leg IK and a ROS 2 action-client node that commands the full quadruped (`quadruped_IK_commander_2.py`) |
| [`lab3-flapping-wing-attitude-control/`](lab3-flapping-wing-attitude-control) | Bio-inspired flight | Cascaded PID attitude and attitude-rate controllers for a flapping-wing robot |
| [`lab4-reinforcement-learning/`](lab4-reinforcement-learning) | Reinforcement learning | Tabular Q-learning (dice game), then DQN, PPO and DDPG agents trained to drive a wheeled robot to a target, with a custom reward function |
| [`critical-review/`](critical-review) | Research review | Critical review of *High-slope terrain locomotion for torque-controlled quadruped robots* |

Each lab folder contains our lab report (PDF) with the method, results and discussion.

## Run it

**ROS 2 labs (1–2):** inside a ROS 2 workspace with the course's quadruped Gazebo simulation running:

```bash
python3 lab2-quadruped-inverse-kinematics/quadruped_IK_commander_2.py --help
```

**RL lab (4):** open `lab4-reinforcement-learning/Session2/trainDDPGrobot.mlx` in MATLAB.
Trained agents (`trainedDQN.mat`, `trainedPPO.mat`, `trainedDDPG.mat`) are included, so the
results can be replayed without retraining.

## Team

Group project by **Abdalrahman Nasser**, **Emil Söderström** and **Pontus Öhman**.
