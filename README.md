Medical Robotics Project
Control and Trajectory Design for Robotic-Assisted Minimally Invasive Surgery

This project explores robot modeling, control, and trajectory planning for a planar robotic manipulator designed for robot-assisted minimally invasive surgery. The work integrates kinematic modeling, dynamic modeling, and advanced control strategies to achieve safe and precise robotic manipulation in surgical environments. 

Read the complete detailed report here:
[Medical_Robotics_Report_Marko.pdf](https://github.com/user-attachments/files/25765417/Medical_Robotics_Report_Marko.pdf)



The project was developed as part of the Medical Robotics course at the University of Coimbra (DEEC).

Keypoints: 

Forward and inverse kinematics

Jacobian-based motion control

Dynamic modeling using the Lagrangian formulation

Gravity compensation

Cartesian force control

Task-space P and PD control

Compliance control with virtual walls

Impedance and admittance control

Trajectory tracking (elliptical and diagonal)

Obstacle avoidance for a 3-link surgical robot

Simulations were implemented using MATLAB and Simulink.

Robot Model

The main system is a 2-DOF planar manipulator, representing a simplified surgical robotic arm.

Forward Kinematics

The end-effector position is computed as:

x = l1 cos(q1) + l2 cos(q1 + q2)
y = l1 sin(q1) + l2 sin(q1 + q2)

These equations map joint angles to Cartesian coordinates, enabling trajectory planning and control. 

Medical_Robotics_Report_Marko

Jacobian Matrix

The Jacobian relates joint velocities to end-effector velocities:

J =
[ -l1 sin(q1) - l2 sin(q1+q2)   -l2 sin(q1+q2)
   l1 cos(q1) + l2 cos(q1+q2)    l2 cos(q1+q2) ]

It is used to transform Cartesian forces into joint torques. 

Medical_Robotics_Report_Marko

Robot Dynamics

The robot dynamics follow the standard manipulator equation:

M(q) q̈ + C(q,q̇) q̇ + g(q) = τ

Where:

M(q) – inertia matrix

C(q,q̇) – Coriolis and centrifugal forces

g(q) – gravity vector

τ – joint torques

The dynamic model was derived using the Lagrangian formulation and implemented in MATLAB functions.

Example functions included:

InertiaMatrix.m

CoriolisMatrix.m

GravityVector.m

computeAcceleration.m

For example, the gravity vector implementation computes:

G1 = g*m2*(lc2*sin(q1+q2) + l1*sin(q1)) + g*m1*lc1*sin(q1)
G2 = g*m2*lc2*sin(q1+q2)

GravityVector

Implemented Control Strategies
1. Gravity Compensation

Torque input:

τ = g(q)

This cancels gravitational forces, allowing the robot to remain stationary at its configuration. 

Medical_Robotics_Report_Marko

2. Cartesian Force Control

Cartesian forces applied at the end-effector are converted to joint torques using:

τ = Jᵀ Fc + g(q)

Tested force vectors:

[0, -10]

[10, 0]

[10, -10]

Results showed that the robot’s final configuration corresponds to the direction and magnitude of the applied force. 

Medical_Robotics_Report_Marko

3. Task-Space Position Control

Two controllers were implemented:

P Controller
τ = Jᵀ(Kp (xd - xa)) + g(q)
PD Controller
τ = Jᵀ(Kp (xd - xa) + Kd (vd - va)) + g(q)

The robot tracks an elliptical trajectory defined by:

xd(t) = 0.15 cos(2πft)
yd(t) = 0.1 sin(2πft)

with frequency f = 1/3 Hz.

The PD controller showed significantly improved accuracy and stability compared to the P controller. 

Medical_Robotics_Report_Marko

4. Compliance Control

Compliance control allows the robot to respond to external forces (e.g., surgeon interaction) while following a trajectory.

Control law:

τ = Jᵀ(Kp(xd-xa) + Kd(vd-va) + Ks fext + fwall) + g(q)

A virtual wall prevents the robot from entering restricted areas.

Key observations:

Smooth response in free space

Oscillations near boundaries if stiffness is too high

Gain tuning required for stable behavior

5. Impedance Control

Impedance control regulates the relationship between force and motion:

Fimp = Md ẍd + Dd(ẋd - ẋa) + Kp(xd - xa) + Fext
τ = Jᵀ Fimp

Used to control motion along a diagonal constraint (y = x).

This approach allows safe interaction with the surgical environment. 

Medical_Robotics_Report_Marko

6. Admittance Control

Admittance control was implemented to adapt robot motion to external forces while following an elliptical trajectory.

Initial MATLAB implementation showed oscillations, so the system was refined with:

improved parameter tuning

trajectory smoothing

Python implementation for improved flexibility

This resulted in stable and accurate trajectory tracking. 

Medical_Robotics_Report_Marko

Trajectory Planning with Obstacle Avoidance

A 3-link planar robot was also implemented for surgical trajectory planning.

Features:

Straight-line motion planning

Detection of a vital area (obstacle)

Reactive path modification

Visualization of robot motion

Implemented modules:

InverseKinematics.m

ForwardKinematics3.m

trajectoryGenerator3.m

DisplayRobot3.m

The robot dynamically adjusts its path to avoid the vital area while maintaining the desired trajectory. 


The project includes simulations for:

robot free fall under gravity

gravity compensation

Cartesian force control

elliptical trajectory tracking

diagonal trajectory control

compliance interaction with virtual walls

impedance control under constraints

admittance control under external forces

These simulations demonstrate robot stability, precision, and interaction behavior.

Technologies Used

MATLAB

Simulink

Python

Robotics dynamics modeling

Lagrangian mechanics

Control theory

References

M. Spong, S. Hutchinson, M. Vidyasagar — Robot Modeling and Control

W. Khalil, E. Dombre — Modeling, Identification and Control of Robots

R. Cortesão — Medical Robotics Course Notes 

tp_RM2024

Author

Marko Kuzmanoski
University of Ljubljana, Faculty of Electrical Engineering
University of Coimbra, Department of Electrical and Computer Engineering
