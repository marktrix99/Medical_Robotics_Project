function Cg = controlCartesian(q)
    global F_ext

    q1 = q(1);
    q2 = q(2);

    G = GravityVector(q);

    % Compute Jacobian
    J = Jacobian_ee(q);
    
    % Ensure Fc is a column vector
    F_ext = F_ext(:);
    
    % Compute joint torques due to Cartesian forces
    tau_Fc = J' * F_ext;

    % Compute control torques
    Cg = G + tau_Fc;
end