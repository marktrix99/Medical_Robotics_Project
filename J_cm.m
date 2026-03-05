function [Jvc1, Jvc2, Jw1, Jw2] = J_cm(q)
    % Define link lengths
    global lc1 lc2 l1;

    % Extract q1 and q2 from q
    q1 = q(1);
    q2 = q(2);
    
    % Linear velocity Jacobian for the first link (3x2 matrix)
    Jvc1 = [-lc1 * sin(q1), 0; 
             lc1 * cos(q1), 0; 
             0, 0];  % This is a 3x2 matrix
    
    % Linear velocity Jacobian for the second link (3x2 matrix)
    Jvc2 = [-l1 * sin(q1) - lc2 * sin(q1 + q2), -lc2 * sin(q1 + q2); 
             l1 * cos(q1) + lc2 * cos(q1 + q2), lc2 * cos(q1 + q2); 
             0, 0];  % This is a 3x2 matrix
    
    % Angular velocity Jacobian for the first link (3x1 matrix)
    Jw1 = [0; 
           0; 
           1];  % This is a 3x1 matrix
    
    % Angular velocity Jacobian for the second link (3x1 matrix)
    Jw2 = [0; 
           0; 
           1];  % This is a 3x1 matrix
end
