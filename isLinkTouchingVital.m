function touching = isLinkTouchingVital(link_start, link_end, vital_center, vital_radius)
% Checks if a link intersects the vital area.
% Inputs:
%   link_start - [x, y] starting point of the link
%   link_end   - [x, y] ending point of the link
%   vital_center - [x, y] center of the vital area
%   vital_radius - radius of the vital area
% Output:
%   touching - true if the link intersects the vital area, false otherwise

    % Vector math for line-circle intersection
    start_to_center = vital_center - link_start(:);
    link_vector = link_end(:) - link_start(:);
    link_length = norm(link_vector);
    link_unit = link_vector / link_length; % Normalize the link vector
    projection_length = dot(start_to_center, link_unit); % Projection onto the link

    % Closest point on the link to the circle center
    if projection_length < 0
        closest_point = link_start(:);
    elseif projection_length > link_length
        closest_point = link_end(:);
    else
        closest_point = link_start(:) + projection_length * link_unit;
    end

    % Check if the closest point is within the circle
    touching = norm(closest_point - vital_center(:)) <= vital_radius;
end
