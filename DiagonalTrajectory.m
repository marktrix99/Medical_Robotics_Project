function dynamicsVector = DiagonalTrajectory(t)
    x_start = 0.25; y_start = 0.2;
    vx = 0.05; vy = 0.05;
    x_d = [x_start + vx * t; y_start + vy * t];
    dx_d = [vx; vy];
    ddx_d = [0; 0];
    dynamicsVector = [x_d;dx_d;ddx_d];
end
