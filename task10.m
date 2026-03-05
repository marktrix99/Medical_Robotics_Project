
% Initialize global parameters
global K B M l1 l2
K = diag([100, 100]);  % Stiffness
B = diag([10, 10]);    % Damping
M = diag([1, 1]);      % Inertia
l1 = 0.3; l2 = 0.2;    % Link lengths

% Time vector
t = linspace(0, 5, 500);

% Desired diagonal trajectory
x_d = 0.2 + 0.1 * t;
y_d = 0.2 + 0.05 * t;
dx_d = gradient(x_d, t);
dy_d = gradient(y_d, t);
ddx_d = gradient(dx_d, t);
ddy_d = gradient(dy_d, t);

% Initial conditions
q = [pi/4; pi/6]; dq = [0; 0];

% Simulate robot dynamics
q_history = []; x_actual = []; y_actual = [];
for i = 1:length(t)
    % Inputs to ImpedanceControl
    inputs = [q(1), q(2), dq(1), dq(2), x_d(i), y_d(i), dx_d(i), dy_d(i), ddx_d(i), ddy_d(i)];
    tau = ImpedanceControl(inputs);

    % Compute joint accelerations
    q_dd = computeAcceleration([q; dq; tau]);

    % Update states using Euler integration
    dq = dq + q_dd * (t(2) - t(1));
    q = q + dq * (t(2) - t(1));

    % Log results
    q_history = [q_history, q];
    pos = ForwardKinematics(q);
    x_actual = [x_actual, pos(1)];
    y_actual = [y_actual, pos(2)];
end

% Plot results
figure;
plot(x_d, y_d, 'r--', 'LineWidth', 1.5); hold on;
plot(x_actual, y_actual, 'b-', 'LineWidth', 1.5);
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Desired vs Actual Trajectory');
legend('Desired', 'Actual');
grid on;

 