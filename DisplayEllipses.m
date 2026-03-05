function DisplayEllipses(input)
    % DisplayEllipses - Visualization of desired vs actual trajectory
    % Inputs:
    %   input - A vector where:
    %       input(1:2) - Desired position [xd; yd]
    %       input(3:4) - Joint angles [q1; q2]

    % Global parameters
    global l1 l2

    % Validate input size
    if numel(input) ~= 4
        error('Input vector must have 4 elements: [xd, yd, q1, q2].');
    end

    % Extract inputs
    DesiredPosition = input(1:2);  % Desired position [xd; yd]
    q = input(3:4);               % Joint angles [q1; q2]

    % Link lengths
    l1 = 0.3;  % Length of link 1
    l2 = 0.2;  % Length of link 2

    % Extract joint angles
    q1 = q(1);
    q2 = q(2);

    % Compute actual position (Forward Kinematics)
    x_actual = l1 * cos(q1) + l2 * cos(q1 + q2);
    y_actual = l1 * sin(q1) + l2 * sin(q1 + q2);
    ActualPosition = [x_actual; y_actual];

    % Persistent variables to store trajectory
    persistent desired_trajectory actual_trajectory fig;
    if isempty(desired_trajectory)
        desired_trajectory = [];
        actual_trajectory = [];
        fig = figure('Name', 'Trajectory Visualization', 'NumberTitle', 'off');
    elseif ~isvalid(fig)
        % Recreate the figure if it was closed
        fig = figure('Name', 'Trajectory Visualization', 'NumberTitle', 'off');
    end

    % Update trajectory
    desired_trajectory = [desired_trajectory; DesiredPosition'];
    actual_trajectory = [actual_trajectory; ActualPosition'];

    % Plot the trajectories
    figure(fig); clf; hold on;

    % Plot desired trajectory (history)
    plot(desired_trajectory(:, 1), desired_trajectory(:, 2), 'r--', 'LineWidth', 1.5, 'DisplayName', 'Desired Trajectory');

    % Plot actual trajectory (history)
    plot(actual_trajectory(:, 1), actual_trajectory(:, 2), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Actual Trajectory');

    % Plot current desired position
    plot(DesiredPosition(1), DesiredPosition(2), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'DisplayName', 'Desired Position');

    % Plot current actual position
    plot(ActualPosition(1), ActualPosition(2), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b', 'DisplayName', 'Actual Position');

    % Plot robot configuration
    plot_robot(q);

    % Add labels, title, and legend
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    title('Desired vs Actual End-Effector Trajectory');
    legend('Location', 'best');
    grid on;
end
