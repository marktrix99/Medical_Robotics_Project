function out = ForwardKinematics3(inputs)
% Computes the Cartesian position (x, y) of the end-effector
% given the joint angles (q1, q2, q3).
% Uses global variables for the link lengths.
% Inputs:
%   inputs - A vector containing the joint angles [q1; q2; q3]
% Outputs:
%   out    - A vector containing the Cartesian position [x; y]

    % Extract joint angles
    q1 = inputs(1);
    q2 = inputs(2);
    q3 = inputs(3);

    % Declare global variables for link lengths
    global l1 l2 l3

    % Validate global variables
    if isempty(l1) || isempty(l2) || isempty(l3)
        error('Link lengths l1, l2, and l3 must be defined as global variables.');
    end

    % Compute the x position
    x = l1 * cos(q1) + l2 * cos(q1 + q2) + l3 * cos(q1 + q2 + q3);

    % Compute the y position
    y = l1 * sin(q1) + l2 * sin(q1 + q2) + l3 * sin(q1 + q2 + q3);

    % Output Cartesian position
    out = [x; y];
end
