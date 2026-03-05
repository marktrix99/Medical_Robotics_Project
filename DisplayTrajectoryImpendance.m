function DisplayTrajectoryImpendance(q)
    % DisplayTrajectoryActual - Visualization of the robot's actual trajectory
    % Inputs:
    %   q - Joint angles [q1; q2]

    % Global parameters
    global l1 l2

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

    % Persistent variable to store trajectory
    persistent actual_trajectory fig;
    if isempty(actual_trajectory)
        actual_trajectory = [];
        % Initialize the figure if it doesn't exist or is invalid
        fig = figure('Name', 'Actual Trajectory', 'NumberTitle', 'off');
    elseif ~isvalid(fig)
        % Recreate the figure if it was closed
        fig = figure('Name', 'Actual Trajectory', 'NumberTitle', 'off');
    end

    % Update trajectory
    actual_trajectory = [actual_trajectory; ActualPosition'];

    % Plot the trajectory
    figure(fig); clf; hold on;

    % Plot actual trajectory (history)
    plot(actual_trajectory(:, 1), actual_trajectory(:, 2), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Actual Trajectory');

    % Plot current position
    plot(ActualPosition(1), ActualPosition(2), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b', 'DisplayName', 'Current Position');

    % Add labels and title
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    title('Actual End-Effector Trajectory');
    legend('Location', 'best');
    grid on;
end
