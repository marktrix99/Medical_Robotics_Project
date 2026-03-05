function tau = ImpedanceControl(inputs)
    global l1 l2 Kp Kd Md Dd Fx_ext Fy_ext Ke Xe

    % Parse inputs
    Xdesired = inputs(1); % Desired X position
    Ydesired = inputs(2); % Desired Y position
    Vxdesired = inputs(3); % Desired X velocity
    Vydesired = inputs(4); % Desired Y velocity
    Axdesired = inputs(5); % Desired X acceleration
    Aydesired = inputs(6); % Desired Y acceleration
    q1 = inputs(7); dq1 = inputs(9); % Joint angles and velocities
    q2 = inputs(8); dq2 = inputs(10);

    % External forces (filtered for stability)
    F_ext = [Fx_ext; Fy_ext]; % Simulated external forces
    F_ext = F_ext - 0.1 * F_ext; % Apply basic filtering

    % Joint variables
    q = [q1; q2];
    dq = [dq1; dq2];

    % Jacobian and position calculation
    J = Jacobian_ee(q); % Jacobian of the end-effector
    x_real = [l1 * cos(q1) + l2 * cos(q1 + q2); l1 * sin(q1) + l2 * sin(q1 + q2)];
    dx_real = J * dq; % Current velocity

    % Desired trajectory
    Pos_desired = [Xdesired; Ydesired];
    Vel_desired = [Vxdesired; Vydesired];
    Accel_desired = [Axdesired; Aydesired];

    % Simplified diagonal constraint
    Pos_desired(2) = Pos_desired(1);
    Vel_desired(2) = Vel_desired(1);
    Accel_desired(2) = Accel_desired(1);

    % Adaptive Gains for Precision
    position_error = norm(Pos_desired - x_real);
    velocity_error = norm(Vel_desired - dx_real);

    % Dynamically adjust gains based on error
    adaptive_Kp = Kp + diag([500, 500]) * position_error;
    adaptive_Kd = Kd + diag([50, 50]) * velocity_error;

    % Compute position and velocity errors
    PosErr = adaptive_Kp * (Pos_desired - x_real); % Position error
    VelErr = adaptive_Kd * (Vel_desired - dx_real); % Velocity error

    % Compute impedance forces
    F_imp = Md * Accel_desired + Dd * VelErr + PosErr + F_ext;

    % Compute torques with gravity and Coriolis compensation
    tau = J' * F_imp + GravityVector(q) + CoriolisMatrix(q, dq) * dq;
end
