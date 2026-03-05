function tau = AdmittanceControl(inputs)
    global Ma Da Ka F_ext x_previous l1 l2

    % Extract inputs
    q = inputs(1:2);    % Current joint angles [q1; q2]
    dq = inputs(3:4);   % Current joint velocities [dq1; dq2]
    x_des_raw = inputs(5:6); % Raw desired Cartesian position [x_des; y_des]

    % Initialize smoothing memory
    if isempty(x_previous)
        x_previous = x_des_raw;
    end

    % Smooth desired position
    alpha = 0.1; % Smoothing factor
    x_des = alpha * x_des_raw + (1 - alpha) * x_previous;
    x_previous = x_des;

    % Compute current end-effector position and velocity
    x = ForwardKinematics(q);
    J = Jacobian_ee(q);
    x_dot = J * dq;

    % Position and velocity error
    e_x = x_des - x;

    % Dynamic scaling of gains based on distance to target
    scaling_factor = max(0.1, 1 - norm(e_x) / 0.5);
    Ka_scaled = Ka * scaling_factor;
    Da_scaled = Da * scaling_factor;

    % Compute desired force with damping and smoothing
    F_des = Ka_scaled * e_x - Da_scaled * x_dot + F_ext;

    % Clamp desired forces to prevent aggressive corrections
    max_force = 5;
    F_des = max(min(F_des, max_force), -max_force);

    % Compute joint torques
    G = GravityVector(q); % Gravity compensation
    tau = J' * F_des + G; % Joint torques
end
