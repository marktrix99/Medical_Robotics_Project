function out = DisplayTrajectoryAdmittance(inputs)
    global l1 l2
    persistent x_desired y_desired x_actual y_actual;

    % Initialize persistent variables
    if isempty(x_desired)
        x_desired = [];
        y_desired = [];
        x_actual = [];
        y_actual = [];
    end

    % Parse inputs
    Xdesired = inputs(1); % Desired X position
    Ydesired = inputs(2); % Desired Y position
    q1 = inputs(3); % Joint angle q1
    q2 = inputs(4); % Joint angle q2

    % Real end-effector position
    x0 = 0; % Base position
    y0 = 0;
    x1 = l1 * cos(q1);
    y1 = l1 * sin(q1);
    x2 = x1 + l2 * cos(q1 + q2);
    y2 = y1 + l2 * sin(q1 + q2);

    % Smoothing the desired trajectory (to align visually)
    smoothing_factor = 0.1; % Adjust this for more smoothing
    if ~isempty(x_desired)
        Xdesired = smoothing_factor * Xdesired + (1 - smoothing_factor) * x_desired(end);
        Ydesired = smoothing_factor * Ydesired + (1 - smoothing_factor) * y_desired(end);
    end

    % Update desired and actual trajectories
    x_desired = [x_desired, Xdesired];
    y_desired = [y_desired, Ydesired];
    x_actual = [x_actual, x2];
    y_actual = [y_actual, y2];

    % Display the robot and trajectories
    figure(1); clf; hold on;

    % Plot trace of desired trajectory (lighter red)
    plot(x_desired, y_desired, 'Color', [1, 0.6, 0.6], 'LineWidth', 1.5, ...
         'DisplayName', 'Desired Trajectory Trace');

    % Plot trace of actual trajectory (blue dashed line)
    plot(x_actual, y_actual, 'b--', 'LineWidth', 1.5, 'DisplayName', 'Actual Trajectory');

    % Highlight the current desired position (dark red)
    plot(Xdesired, Ydesired, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r', ...
         'DisplayName', 'Current Desired Position');

    % Plot robot links
    plot([x0, x1], [y0, y1], 'k-', 'LineWidth', 2); % First link (black)
    plot([x1, x2], [y1, y2], 'k-', 'LineWidth', 2); % Second link (black)

    % Add joints
    plot(x0, y0, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k'); % Base joint
    plot(x1, y1, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k'); % First joint
    plot(x2, y2, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k'); % End-effector joint

    % Format the axes
    axis equal;
    axis([-1.5 2 -1.5 2]); % Adjusted aspect ratio and limits
    xlabel('X Position');
    ylabel('Y Position');
    title('Robot Trajectory and Configuration');
    legend('Location', 'best');
    grid on;

    % Refresh the plot
    drawnow;
    hold off;

    % Output dummy value
    out = 0;
end
