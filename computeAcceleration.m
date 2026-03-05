function q_dd = computeAcceleration(inputVector)
    global Fv
    % Extract q, dq, tau from the input vector (assumes input is a 6x1 vector)
    q = inputVector(1:2);    % q is a 2x1 vector [q1; q2]
    dq = inputVector(3:4);   % dq is a 2x1 vector [dq1; dq2]
    tau = inputVector(5:6);  % tau is a 2x1 vector [tau1; tau2]
    
    % Debugging inputs
    disp('Inputs:');
    disp('q:'); disp(q);
    disp('dq:'); disp(dq);
    disp('tau:'); disp(tau);

    % Compute the mass matrix M(q)
    Mq = InertiaMatrix(q);
    disp('Mq:'); disp(Mq);

    % Compute Coriolis/centrifugal forces
    Cq = CoriolisMatrix(q, dq);
    disp('Cq:'); disp(Cq);

    % Compute Gravity Vector
    Gq = GravityVector(q);
    disp('Gq:'); disp(Gq);


    % Compute joint accelerations
    q_dd = inv(Mq) * (tau - Cq * dq - Gq - Fv * dq);
end
