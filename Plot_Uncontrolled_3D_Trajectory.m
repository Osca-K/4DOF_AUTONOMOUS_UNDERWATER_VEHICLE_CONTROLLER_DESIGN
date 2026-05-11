% ChatGPT was also used to assint with this code to plot 3D Trajectory of the UnconrolledAUV

clear; clc; close all;


%% Create Results folder if it does not exist
if ~exist('Results','dir')
    mkdir('Results');
end

%% Load latest trim point and model data
[A, B, C, D, x_trim, U_trim] = auv_4dof_linear_model();

%% Simulation time
Tfinal = 120;              % seconds
tspan = [0 Tfinal];

%% Open-loop (uncontrolled input)

% Actuator inputs constant
U_open = U_trim;

%% Initial condition
% Option 1: Start near the trim condition at survey depth
x0 = x_trim;
x0(1) = 0;      % initial X
x0(2) = 0;      % initial Y
x0(3) = -20;    % initial Z (depth)

% small perturbation so the open-loop Path
x0(4) = x0(4) + deg2rad(2);   % small heading offset
x0(6) = x0(6) + 0.01;         % small sway perturbation



%% Simulate nonlinear open-loop model
odefun = @(t, x) auv_4dof_nonlinear(t, x, U_open);
[t, x] = ode45(odefun, tspan, x0);

%% Extract actual trajectory
X = x(:,1);
Y = x(:,2);
Z = x(:,3);

%% Desired reference trajectory (for visual comparison only)
% This is an ideal circular sweep at constant depth.
u0   = x_trim(5);
r0   = x_trim(8);
psi0 = x0(4);

R = u0 / r0;    % sweep radius

X_des = x0(1) + R*(sin(psi0 + r0*t) - sin(psi0));
Y_des = x0(2) - R*(cos(psi0 + r0*t) - cos(psi0));
Z_des = x_trim(3)*ones(size(t));   % desired constant-depth circle

%% 3D plot
figure;
plot3(X, Y, Z, 'r-', 'LineWidth', 2); hold on;
plot3(X_des, Y_des, Z_des, 'k--', 'LineWidth', 2);

% Mark start and end points
plot3(X(1), Y(1), Z(1), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 8);
plot3(X(end), Y(end), Z(end), 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 8);

grid on;
hx = xlabel('x [m]');
hy = ylabel('y [m]');
hz = zlabel('z [m]');
ht = title('Uncontrolled 3D AUV Trajectory');
set([hx, hy, hz, ht], 'FontWeight', 'normal', 'FontName', 'Times New Roman');
legend("AUV's trajectory", 'Desired trajectory', 'Start', 'End', 'Location', 'northeast');
view(40, 20);

saveas(gcf, fullfile('Results', 'Uncontrolled_3D_Trajectory.png'));

t
figure;
plot(X, Y, 'r-', 'LineWidth', 2); hold on;
plot(X_des, Y_des, 'k--', 'LineWidth', 2);
plot(X(1), Y(1), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 8);
plot(X(end), Y(end), 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 8);
grid on;
hxl = xlabel('x [m]');
hyl = ylabel('y [m]');
htl = title('Top View of Uncontrolled AUV Trajectory');
set([hxl, hyl, htl], 'FontWeight', 'normal', 'FontName', 'Times New Roman');
legend("AUV's trajectory", 'Desired trajectory', 'Start', 'End', 'Location', 'northeast');
axis equal;

saveas(gcf, fullfile('Results', 'Uncontrolled_Top_View.png'));