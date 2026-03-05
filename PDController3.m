function tau = PDController3(inputs)
% PD Controller for three joints
% Inputs:
%   q      - Current joint angles [q1; q2; q3] (radians)
%   q_des  - Desired joint angles [q1_des; q2_des; q3_des] (radians)
%   dq     - Current joint velocities [dq1; dq2; dq3] (rad/s)
%   dq_des - Desired joint velocities [dq1_des; dq2_des; dq3_des] (rad/s)
% Output:
%   tau    - Control torques for the joints [tau1; tau2; tau3]
q=inputs(1:3);
q_des=inputs(4:6);
dq=inputs(7:9);
dq_des=inputs(10:12);
global Kp Kd 
% Compute position error
e_pos = q_des - q;

% Compute velocity error
e_vel = dq_des - dq;

% PD control law
tau = Kp .* e_pos + Kd .* e_vel;
end
