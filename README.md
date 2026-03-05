# Control and Trajectory Design for Robotic-Assisted Minimally Invasive Surgery

This project explores robot modeling, control, and trajectory planning for a planar robotic manipulator designed for robot-assisted minimally invasive surgery.
It integrates kinematic modeling, dynamic modeling, and advanced control strategies to achieve safe and precise robotic manipulation in surgical environments.

## 📄 Complete detailed report: 

[Medical_Robotics_Report_Marko.pdf](https://github.com/user-attachments/files/25765509/Medical_Robotics_Report_Marko.pdf)

The project was developed as part of the Medical Robotics course at the University of Coimbra (DEEC).

### Key Features:

* Forward and inverse kinematics  
* Jacobian-based motion control  
* Dynamic modeling using the Lagrangian formulation  
* Gravity compensation  
* Cartesian force control  
* Task-space P and PD control  
* Compliance control with virtual walls  
* Impedance and admittance control  
* Trajectory tracking (elliptical and diagonal)  
* Obstacle avoidance for a 3-link surgical robot


Simulations were implemented using MATLAB and Simulink.

## Robot Model:
The system is a 2-DOF planar manipulator, representing a simplified surgical robotic arm.

### Forward Kinematics

The end-effector position is given by:

```math
x = l_1 \cos(q_1) + l_2 \cos(q_1 + q_2)
y = l_1 \sin(q_1) + l_2 \sin(q_1 + q_2)
```

### Jacobian

The Jacobian relates joint velocities to end-effector velocities:

```math
J = [ -l1 sin(q1) - l2 sin(q1+q2)   -l2 sin(q1+q2)
      l1 cos(q1) + l2 cos(q1+q2)     l2 cos(q1+q2) ]
```
### Robot Dynamics

The manipulator dynamics follow:
```math
M(q) q̈ + C(q,q̇) q̇ + g(q) = τ
```
Where:

M(q) – inertia matrix

C(q,q̇) – Coriolis and centrifugal forces

g(q) – gravity vector

τ – joint torques

These were implemented in MATLAB functions such as:

- `InertiaMatrix.m`
- `CoriolisMatrix.m`
- `GravityVector.m`
- `Jacobian_ee.m`
- `computeAcceleration.m`

## Control Strategies

The project implements several control approaches relevant to surgical robotics:

* Gravity compensation to counteract gravitational forces

* Cartesian force control using Jacobian transpose

* Task-space P and PD controllers for trajectory tracking

* Compliance control with virtual wall constraints

* Impedance control for constrained motion along a diagonal

* Admittance control for adapting robot motion to external forces

* Trajectory tracking experiments include elliptical and diagonal paths.

* Trajectory Planning with Obstacle Avoidance

* A 3-link planar robot was implemented for surgical trajectory planning with obstacle avoidance.

Features include:

* Straight-line motion planning
* Detection of a vital area (obstacle)
* Reactive trajectory adjustment
* Robot motion visualization

  
![task7](https://github.com/user-attachments/assets/d3de38ec-49c6-4616-96e5-69e4a017d946)
![task9](https://github.com/user-attachments/assets/8ab02296-908d-4fcf-bdad-7fb9098d414b)


Author
Marko Kuzmanoski
University of Ljubljana – Faculty of Electrical Engineering
University of Coimbra – Department of Electrical and Computer Engineering
