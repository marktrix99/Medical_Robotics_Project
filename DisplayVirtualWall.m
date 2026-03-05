function DisplayVirtualWall(inputs)
    % DISPLAYVIRTUALWALL - Visualize the 2-link planar robot interacting
    % with a virtual wall far to the right and its trajectory.
    %
    % Inputs:
    %   inputs - Vector containing:
    %       [q1, q2, X_actual, Y_actual, F_wall_x, F_wall_y]
    %
    % Global Variables:
    %   X_min, X_max, Y_min, Y_max - Define the virtual wall boundaries
    %   l1, l2 - Robot link lengths

    % Declare global variables
    global X_min X_max Y_min Y_max l1 l2

    % Extract inputs
    q1 = inputs(1); % Joint angle 1
    q2 = inputs(2); % Joint angle 2
    X_actual = inputs(3); % End-effector X position
    Y_actual = inputs(4); % End-effector Y position
    F_wall = inputs(5:6); % [F_wall_x, F_wall_y]

    % Compute joint positions (forward kinematics)
    joint1 = [0; 0]; % Base of the robot
    joint2 = [l1 * cos(q1); l1 * sin(q1)]; % Position of the first joint
    end_effector = [joint2(1) + l2 * cos(q1 + q2); joint2(2) + l2 * sin(q1 + q2)]; % End-effector position

    % Persistent variables for the figure and trajectory
    persistent fig trajectory;

    % Initialize the figure and trajectory on first call
    if isempty(fig) || ~isvalid(fig)
        fig = figure('Name', 'Robot and Wall Interaction', 'NumberTitle', 'off');
        trajectory = []; % Reset trajectory history
    end

    % Update the trajectory
    trajectory = [trajectory; X_actual, Y_actual];

    % Plot the robot and interaction
    figure(fig); clf; hold on;

    % Plot the virtual wall as a much larger shaded area (further to the right)
    wall_offset = 0.95; % Shift the wall further to the right
    wall_width = 0.8;  % Increase the wall width
    wall_height_extension = 2; % Extend the wall height significantly
    fill([X_max + wall_offset, X_max + wall_offset + wall_width, ...
          X_max + wall_offset + wall_width, X_max + wall_offset], ...
         [Y_min - wall_height_extension, Y_min - wall_height_extension, ...
          Y_max + wall_height_extension, Y_max + wall_height_extension], ...
         [0.8 0.8 0.8], 'FaceAlpha', 0.5, 'EdgeColor', 'none', ...
         'DisplayName', 'Virtual Wall');

    % Define the uniform color for both links
    link_color = [0.2 0.4 0.8]; % Blue shade for both links

    % Plot the robot configuration
    % Plot first link
    plot([joint1(1), joint2(1)], [joint1(2), joint2(2)], '-o', ...
         'Color', link_color, 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor', link_color, 'DisplayName', 'Link 1');
    % Plot second link
    plot([joint2(1), end_effector(1)], [joint2(2), end_effector(2)], '-o', ...
         'Color', link_color, 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor', link_color, 'DisplayName', 'Link 2');

    % Plot the end-effector position
    plot(X_actual, Y_actual, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k', 'DisplayName', 'End-Effector');

    % Plot the force applied by the virtual wall (arrow)
    quiver(X_actual, Y_actual, F_wall(1), F_wall(2), 0.2, 'g', 'LineWidth', 2, 'MaxHeadSize', 1, ...
           'DisplayName', 'Wall Force');

    % Plot the trajectory of the end-effector
    plot(trajectory(:, 1), trajectory(:, 2), 'm-', 'LineWidth', 1.5, 'DisplayName', 'Trajectory');

    % Add labels, legend, and grid
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    title('Planar Robot with Virtual Wall Interaction');
    axis equal;
    legend('Location', 'best');
    grid on;

    % Set plot limits for better visualization
    % Ensure both links, trajectory, and the very large wall are visible
    arm_reach = l1 + l2; % Maximum reach of the robot
    xlim([-arm_reach - 0.1, X_max + wall_offset + wall_width + 0.5]);
    ylim([-arm_reach - wall_height_extension, arm_reach + wall_height_extension]);
end
