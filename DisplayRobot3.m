function DisplayRobot3(inputs)
% DisplayRobot3 - Visualizes the current configuration of a 3-link planar robot.
% Only displays the robot's current position without retaining past configurations.
%
% Inputs:
%   inputs - A vector containing:
%       - q1, q2, q3: Joint angles (radians)
%       - x3, y3: Cartesian position of the end-effector
%
% Global Variables:
%   vital_center - A 2x1 vector [x; y] specifying the center of the vital area.
%   vital_radius - Radius of the vital area.
%   l1, l2, l3 - Link lengths of the robot.

    % Extract joint angles and end-effector position from inputs
    q1 = inputs(1);
    q2 = inputs(2);
    q3 = inputs(3);
    x3 = inputs(4); % End-effector X position
    y3 = inputs(5); % End-effector Y position

    % Declare global variables
    global l1 l2 l3 vital_center vital_radius;

    % Validate global variables
    if isempty(l1) || isempty(l2) || isempty(l3)
        error('Link lengths l1, l2, and l3 must be defined as global variables.');
    end
    if isempty(vital_center) || isempty(vital_radius)
        error('Vital area parameters must be defined as global variables.');
    end

    % Compute joint positions
    x0 = 0; y0 = 0; % Base position
    x1 = l1 * cos(q1);
    y1 = l1 * sin(q1);
    x2 = x1 + l2 * cos(q1 + q2);
    y2 = y1 + l2 * sin(q1 + q2);

    % Clear the figure to only show the current position
    clf;
    hold on;
    grid on;
    axis equal;
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    title('Current Robot Configuration');
    
    % Plot the vital area
    theta = linspace(0, 2*pi, 100);
    x_vital = vital_center(1) + vital_radius * cos(theta);
    y_vital = vital_center(2) + vital_radius * sin(theta);
    fill(x_vital, y_vital, 'r', 'FaceAlpha', 0.3, 'EdgeColor', 'none', ...
         'DisplayName', 'V2ital Area');

    % Plot the robot links
    plot([x0, x1], [y0, y1], 'k-', 'LineWidth', 2); % Link 1
    plot([x1, x2], [y1, y2], 'b-', 'LineWidth', 2); % Link 2
    plot([x2, x3], [y2, y3], 'r-', 'LineWidth', 2); % Link 3

    % Plot the joints
    plot(x0, y0, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k'); % Base
    plot(x1, y1, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k'); % Joint 1
    plot(x2, y2, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k'); % Joint 2
    plot(x3, y3, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r'); % End-effector

    drawnow;
end
