function X_Y_actual = ForwardKinematics(q)
    global l1 l2
    % FORWARDKINEMATICS - Computes the end-effector position in Cartesian
    % coordinates
    % Outputs:
    %   X_actual - X-coordinate of the end-effector (scalar)
    %   Y_actual - Y-coordinate of the end-effector (scalar)

    % Extract joint angles
    q1 = q(1);
    q2 = q(2);

    % Compute Cartesian position
    X_actual = l1 * cos(q1) + l2 * cos(q1 + q2);
    Y_actual = l1 * sin(q1) + l2 * sin(q1 + q2);

    % Combine X_actual and Y_actual into a single vector
    X_Y_actual = [X_actual; Y_actual];
end
