% Define global parameters for the robot
global m1 m2 lc1 lc2 I1czz I2czz l1 l2 q0 dq0 g tau Fv Fx_ext Fy_ext X_min X_max Y_min Y_max K_wall Kd Md Dd Kp F_ext Ma Da Ka x_previous

% Robot parameters
m1 = 1.2;   % Mass of link 1 (kg)
m2 = 1.0;   % Mass of link 2 (kg)
l1 = 1.0;   % Length of link 1 (m)
l2 = 0.8;   % Length of link 2 (m)
lc1 = 0.5;  % Distance to CoM of link 1 (m)
lc2 = 0.4;  % Distance to CoM of link 2 (m)
I1czz = (1/12) * m1 * l1^2;  % Moment of inertia for link 1
I2czz = (1/12) * m2 * l2^2;  % Moment of inertia for link 2
g = 9.81;   % Gravitational acceleration (m/s^2)
Fv = 0.1;   % Viscous damping coefficient
tau = [0; 0]; % Initialize torque vector

% Compliance control parameters
Kp = diag([500, 500]); % Moderate stiffness for trajectory tracking
Kd = diag([50, 50]);   % Lower damping to avoid overdamping
Md = diag([0.05, 0.05]); % Smaller inertia for faster, stable response
Dd = diag([5, 5]);     % Minimal damping for velocity error

% Virtual wall parameters
X_min = 0.1; % Minimum x-coordinate (m)
X_max = 0.5; % Maximum x-coordinate (m)
Y_min = 0.1; % Minimum y-coordinate (m)
Y_max = 0.4; % Maximum y-coordinate (m)
K_wall = 300; % Moderate stiffness for virtual wall (N/m)

% External forces
Fx_ext = 0.010;  % External force in x direction (N)
Fy_ext = -0.010; % External force in y direction (N)
F_ext  = [Fx_ext;Fy_ext];


%Admittance control Parameters
Ma = diag([0.1, 0.1]); % Virtual mass
Ka = diag([10.0, 10.0]); % Reduced stiffness
Da = 2 * sqrt(Ma * Ka); % Critically damped
% Initialize previous desired position
x_previous = [];

% Initial conditions
q0 = [pi/4; pi/6];  % Joint angles (rad)
dq0 = [0; 0];   % Joint velocities (rad/s)

% Test control functions
Cg = controlCartesian(q0);
Mq = InertiaMatrix(q0);
Cq = CoriolisMatrix(q0, dq0);
Gq = GravityVector(q0);
J = Jacobian_ee(q0);
Acc = computeAcceleration([q0; dq0; tau]);

% Compliance Control tests
X_d = 0;
Y_d = 0;
Vx_d = 0;
Vy_d = 0;
inputs = [X_d; Y_d; Vx_d; Vy_d; q0(1); q0(2); dq0;F_ext];
tau_compliance = ComplianceControl(inputs);

% Forward Kinematics
X_Y_actual = ForwardKinematics(q0);

% Virtual Wall
inputs_wall = [X_Y_actual(1), X_Y_actual(2)];
F_wall = VirtualWallLogic(inputs_wall);

% Impedance Control tests
x_d = [0.3; 0.2]; % Desired end-effector position (m)
dx_d = [0.05; 0.05]; % Desired end-effector velocity (m/s)
ddx_d = [0; 0]; % Desired end-effector acceleration (m/s^2)
inputs_impedance = [x_d; dx_d; ddx_d; q0; dq0];
tau_impedance = ImpedanceControl(inputs_impedance);

% Admittance Control tests
Xdesired = 0.4; Ydesired = 0.3; % Desired position (m)
Vxdesired = 0.1; Vydesired = 0.1; % Desired velocity (m/s)
inputs_admittance = [Xdesired; Ydesired; q0(1); q0(2); dq0(1); dq0(2)];
tau_admittance = AdmittanceControl(inputs_admittance)
