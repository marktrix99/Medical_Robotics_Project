function Gq = GravityVector(q)
    % Define global parameters
    global m1 m2 lc1 lc2 l1 g  % Masses, link lengths, gravitational acceleration
    
    % Extract joint angles
    q1 = q(1);
    q2 = q(2);
    
    % Gravitational forces acting on each link
    G1 = g * m2 * (lc2 * sin(q1 + q2) + l1 * sin(q1)) + g * m1 * lc1 * sin(q1); % Gravity on link 1
    G2 = g * m2 * lc2 * sin(q1 + q2);                                          % Gravity on link 2

    % Combine into the gravity vector
    Gq = [G1; G2];
end
