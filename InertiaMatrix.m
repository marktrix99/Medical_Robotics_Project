function Mq = InertiaMatrix(q)
    % Define the global system parameters
    global m1 m2 lc1 lc2 I1czz I2czz l1 l2


    I1czz = (1/12) * m1 * l1^2;  % Moment of inertia for link 1
    I2czz = (1/12) * m2 * l2^2;  % Moment of inertia for link 2

    % Extract joint angles
    q1 = q(1);  % Joint angle 1
    q2 = q(2);  % Joint angle 2

    % Mass matrix elements
    M11 = m2 * l1^2 + 2 * m2 * cos(q2) * l1 * lc2 + m1 * lc1^2 + m2 * lc2^2 + I1czz + I2czz;
    M12 = m2 * lc2^2 + l1 * m2 * cos(q2) * lc2 + I2czz;
    M21 = M12;  % Symmetry of the mass matrix
    M22 = m2 * lc2^2 + I2czz;

    % Combine elements into the full mass matrix
    Mq = [M11, M12; 
          M21, M22];
    disp('Mq:');
    disp(Mq);  % Debugging output
end
