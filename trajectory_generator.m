% Generate trajectory
t = linspace(0, 10, 100);  % Total simulation time
x_vals = zeros(size(t));
y_vals = zeros(size(t));

for i = 1:length(t)
    output = trajectory_generator3_1(t(i));  % Call the function for trajectory generation
    x_vals(i) = output(1);
    y_vals(i) = output(2);
end

% Plot the trajectory
figure;
plot(x_vals, y_vals, 'b-', 'LineWidth', 2);
hold on;
theta = linspace(0, 2*pi, 100);
x_vital = vital_center(1) + vital_radius * cos(theta);
y_vital = vital_center(2) + vital_radius * sin(theta);
fill(x_vital, y_vital, 'r', 'FaceAlpha', 0.3, 'EdgeColor', 'none', 'DisplayName', 'Vital Area');
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Robot Trajectory Avoiding Vital Area');
grid on;
axis equal;
