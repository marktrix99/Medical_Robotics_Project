function F_wall = VirtualWallLogic(inputs) 
global X_min X_max Y_min Y_max K_wall
%X_actual, Y_actual
 
    % Inputs:
    %   X_actual - Current X position of the end-effector
    %   Y_actual - Current Y position of the end-effector
    %   X_min, X_max - X-axis virtual wall boundaries
    %   Y_min, Y_max - Y-axis virtual wall boundaries
    %   K_wall - Wall stiffness constant
    %
    % Output:
    %   F_wall - 2x1 vector of virtual wall forces [F_wall_x; F_wall_y]

    % Initialize wall forces
    F_wall_x = 0;
    F_wall_y = 0;
    X_actual = inputs(1);
    Y_actual = inputs(2);

    % Check X boundaries
    if X_actual < X_min
        F_wall_x = K_wall * (X_min - X_actual); % Push back into boundary
    elseif X_actual > X_max
        F_wall_x = K_wall * (X_max - X_actual);
    end

    % Check Y boundaries
    if Y_actual < Y_min
        F_wall_y = K_wall * (Y_min - Y_actual); % Push back into boundary
    elseif Y_actual > Y_max
        F_wall_y = K_wall * (Y_max - Y_actual);
    end

    % Combine forces
    F_wall = [F_wall_x; F_wall_y];
end
