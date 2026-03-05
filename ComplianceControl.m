function tau = ComplianceControl(inputs)
    % Inputs: [X_d, Y_d, Vx_d, Vy_d, q1, q2, dq1, dq2, Fx_ext, Fy_ext]
    global l1 l2 Fx_ext Fy_ext X_min X_max Y_min Y_max

    % Gains
    Kp = diag([500, 500]); % Position gain
    Kd = diag([50, 50]);   % Velocity gain
    Ks = diag([100, 100]); % Compliance gain


    % Desired positions and velocities
    X_d = inputs(1);  % Desired x position
    Y_d = inputs(2);  % Desired y position
    Vx_d = inputs(3); % Desired x velocity
    Vy_d = inputs(4); % Desired y velocity

    % Joint angles and velocities
    q1 = inputs(5);
    q2 = inputs(6);
    dq1 = inputs(7);
    dq2 = inputs(8);

    % External forces
    Fx_wall = inputs(9);
    Fy_wall = inputs(10);

    % Compute actual position (Forward Kinematics)
    X_actual = l1 * cos(q1) + l2 * cos(q1 + q2);
    Y_actual = l1 * sin(q1) + l2 * sin(q1 + q2);

    % Compute actual velocity
    J = [-l1*sin(q1)-l2*sin(q1+q2), -l2*sin(q1+q2);
          l1*cos(q1)+l2*cos(q1+q2),  l2*cos(q1+q2)];
    dq = [dq1; dq2];
    V_actual = J * dq;

    % Compute wall forces
    F_wall = [Fx_wall; Fy_wall];
    if X_actual < X_min
        F_wall(1) = 100 * (X_min - X_actual);
    elseif X_actual > X_max
        F_wall(1) = 100 * (X_max - X_actual);
    end
    if Y_actual < Y_min
        F_wall(2) = 100 * (Y_min - Y_actual);
    elseif Y_actual > Y_max
        F_wall(2) = 100 * (Y_max - Y_actual);
    end

    % Compute Cartesian control forces
    F_pos = Kp * ([X_d; Y_d] - [X_actual; Y_actual]);
    F_vel = Kd * ([Vx_d; Vy_d] - V_actual);
    F_compliance = Ks * [Fx_ext; Fy_ext];

    % Total force in Cartesian space
    F_total = F_pos + F_vel + F_compliance + F_wall;

    % Convert to joint torques
    Gq = GravityVector([q1; q2]); % Gravity compensation
    tau = J' * F_total + Gq;
end
