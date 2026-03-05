function output = trajectory_generator3_1(t)
% Generates a smooth trajectory while avoiding the vital area.
% Inputs:
%   - t: Current time
% Outputs:
%   - output: [x, y] Cartesian position of the trajectory at time t

    % Global variables
    global vital_center vital_radius sensor_radius;

    % Start and end points of the trajectory
    x_start = 0; y_start = 1.5; % Initial position (up)
    x_end = 0; y_end = -1.5;    % Final position (down)

    % Total simulation time
    t_total = 10; % Total simulation time

    % Normalize time to [0, 1]
    progress = min(t / t_total, 1);

    % Generate a straight-line trajectory
    x = x_start;
    y = y_start + (y_end - y_start) * progress;

    % Check if the trajectory point is near or inside the sensor radius
    distance_to_vital = norm([x - vital_center(1), y - vital_center(2)]);

    if distance_to_vital <= sensor_radius
        % If near the vital area, generate a detour path around it
        % Calculate the angle between the current point and the vital area center
        theta = atan2(y - vital_center(2), x - vital_center(1));

        % Adjust the trajectory to move outside the vital area
        detour_factor = max(0, (sensor_radius - distance_to_vital) / sensor_radius); % Increase detour with proximity
        detour_radius = vital_radius * (1 + detour_factor * 1.5); % Move further away from the vital area
        x = vital_center(1) + detour_radius * cos(theta);
        y = vital_center(2) + detour_radius * sin(theta);

        % Optionally, you can generate a smoother transition back to the original line
        % once the trajectory has safely passed the vital area (i.e., when it's clear of the sensor_radius).
    end

    % Output the adjusted trajectory point as [x; y]
    output = [x; y];
end
