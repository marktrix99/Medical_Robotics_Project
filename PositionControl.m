% task 7 
function ppd = PositionControl(inputs) %inputs[position;velocity;q;dq]
    % Global parameters
    global l1 l2 

    % Controller gains
    Kp = diag([1000, 1000]);  % Proportional gain (2x2)
    Kd = diag([80, 80]);      % Derivative gain (2x2)

    Xdesired = inputs(1);    % Desired X position
    Ydesired = inputs(2);    % Desired Y position
    Vxdesired = inputs(3);   % Desired X velocity
    Vydesired = inputs(4);   % Desired Y velocity

    q1 = inputs(5);          % Joint angle q1
    q2 = inputs(6);          % Joint angle q2

    dq1 = inputs(7);         % Joint velocity dq1
    dq2 = inputs(8);         % Joint velocity dq2

    % Current joint angles and velocities
    q = [q1; q2];
    dq = [dq1; dq2];

    % Desired position and velocity in Cartesian space
    Positions = [Xdesired; Ydesired];
    Velocities = [Vxdesired; Vydesired];

    % Compute current end-effector position (Forward Kinematics)
    x_current = l1 * cos(q1) + l2 * cos(q1 + q2);
    y_current = l1 * sin(q1) + l2 * sin(q1 + q2);
    RealPosition = [x_current; y_current];

    % Compute current end-effector velocity
    J = Jacobian_ee(q);  % Compute Jacobian matrix
    V_current = J * dq;     % End-effector velocity

    % Gravity vector
    Gq = GravityVector(q);

    % Compute position and velocity errors
    PosErr = Kp * (Positions - RealPosition);  % Proportional error
    VelErr = Kd * (Velocities - V_current);    % Derivative error

    % Compute P and PD control
    ppd = Gq + J' * PosErr + J' * VelErr;
end