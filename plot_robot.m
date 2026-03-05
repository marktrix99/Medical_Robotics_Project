function plot_robot(q)
    global l1 l2

    % Ensure global variables are initialized
    if isempty(l1) || isempty(l2)
        error('Global variables l1 and l2 (link lengths) must be defined.');
    end

    % Extract joint angles
    q1 = q(1);
    q2 = q(2);

    % Compute joint positions
    joint1 = [0; 0];  % Base position
    joint2 = [l1 * cos(q1); l1 * sin(q1)];  % First joint
    end_effector = [joint2(1) + l2 * cos(q1 + q2); joint2(2) + l2 * sin(q1 + q2)];  % End-effector

    % Plot robot links
    plot([joint1(1), joint2(1)], [joint1(2), joint2(2)], '-ko', 'LineWidth', 2, 'MarkerFaceColor', 'k');
    plot([joint2(1), end_effector(1)], [joint2(2), end_effector(2)], '-ko', 'LineWidth', 2, 'MarkerFaceColor', 'k');
end
