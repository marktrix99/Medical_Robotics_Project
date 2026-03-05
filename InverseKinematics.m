function output = InverseKinematics(inputs)
% Computes joint angles for a 3-link planar robot while avoiding the vital area.
% Inputs:
%   - inputs: [x, y] Cartesian target position
% Outputs:
%   - output: [q1, q2, q3] Joint angles

    % Extract input position
    x = inputs(1);
    y = inputs(2);

    % Global variables
    global l1 l2 l3 vital_center vital_radius sensor_radius;

    % Compute the distance from the target position to the vital area center
    distance_to_vital = norm([x - vital_center(1), y - vital_center(2)]);

    % If the target is within the sensor radius, adjust the target position
    if distance_to_vital <= sensor_radius
        % Calculate direction from the target to the vital area center
        theta = atan2(y - vital_center(2), x - vital_center(1));
        
        % Gradually move the target away from the vital area center
        % The closer the target is, the larger the adjustment
        detour_factor = max(0, (sensor_radius - distance_to_vital) / sensor_radius);
        
        % Move the target away from the vital area
        detour_radius = vital_radius * (1 + detour_factor * 2); % Increase the detour based on proximity
        x = vital_center(1) + detour_radius * cos(theta);
        y = vital_center(2) + detour_radius * sin(theta);
    end

    % Check if any link is touching or close to the vital area
    % Calculate positions of the three links' endpoints
    x1 = l1 * cos(0); % First joint (base)
    y1 = l1 * sin(0);

    x2 = x1 + l2 * cos(0); % Second joint (should be based on joint angles)
    y2 = y1 + l2 * sin(0);

    x3 = x2 + l3 * cos(0); % End-effector (should be based on joint angles)
    y3 = y2 + l3 * sin(0);

    % Check if any of the link endpoints are near the vital area
    is_collision = false;
    if norm([x1 - vital_center(1), y1 - vital_center(2)]) <= vital_radius || ...
       norm([x2 - vital_center(1), y2 - vital_center(2)]) <= vital_radius || ...
       norm([x3 - vital_center(1), y3 - vital_center(2)]) <= vital_radius
        is_collision = true;
    end

    if is_collision
        % If collision detected, move all links back
        move_back_distance = vital_radius * 2; % Distance to move back
        % Move the entire arm along the line away from the vital area
        theta = atan2(y - vital_center(2), x - vital_center(1));
        
        % Move back in the opposite direction of the vital area
        x = vital_center(1) - move_back_distance * cos(theta);
        y = vital_center(2) - move_back_distance * sin(theta);
    end

    % Compute the distance to the end-effector from the new position
    r = sqrt(x^2 + y^2);

    % Ensure the target position is within the robot's workspace
    if r > (l1 + l2 + l3)
        error('Target position is outside the robot''s reachable workspace.');
    end

    % Joint angle calculations
    theta1 = atan2(y, x);
    cos_alpha = (l1^2 + r^2 - l2^2) / (2 * l1 * r);
    cos_alpha = min(max(cos_alpha, -1), 1); % Clamp for numerical stability
    alpha = acos(cos_alpha);
    q1 = theta1 - alpha;

    cos_beta = (l1^2 + l2^2 - r^2) / (2 * l1 * l2);
    cos_beta = min(max(cos_beta, -1), 1); % Clamp for numerical stability
    beta = acos(cos_beta);
    q2 = pi - beta;

    % Compute the third joint angle (end-effector orientation)
    q3 = atan2(y - (l1 * sin(q1) + l2 * sin(q1 + q2)), ...
               x - (l1 * cos(q1) + l2 * cos(q1 + q2)));

    % Output the computed joint angles
    output = [q1; q2; q3];
end
