% Updated_Transfer_Functions.m
% Extract and analyze transfer functions from the 4DOF linear model
% 
% This script creates transfer functions from the state-space model
% and analyzes the open-loop dynamics.

clear; clc; close all;

%% Load 4DOF linear model
[A, B, C, D] = auv_4dof_linear_model();

fprintf('\n======== 4DOF REMUS-BASED AUV LINEAR MODEL ========\n');
fprintf('Transfer Function Analysis\n');
fprintf('===================================================\n\n');

%% Create state-space system
sys = ss(A, B, C, D);

%% Create transfer functions
TF = tf(sys);

% Extract individual transfer functions of interest
G_X_XT = TF(1,1);          % X position / thrust
G_Y_dr = TF(2,2);          % Y position / rudder
G_Z_de = TF(3,3);          % Z position / elevator
G_psi_dr = TF(4,2);        % yaw heading / rudder

%% Display transfer functions
fprintf('Transfer Function: X(s) / XT(s)  (Surge Position / Thrust)\n');
fprintf('---------------------------------------------------------\n');
disp(G_X_XT);
fprintf('\n');

fprintf('Transfer Function: Y(s) / delta_r(s)  (Sway Position / Rudder)\n');
fprintf('---------------------------------------------------------------\n');
disp(G_Y_dr);
fprintf('\n');

fprintf('Transfer Function: Z(s) / delta_e(s)  (Heave Position / Elevator)\n');
fprintf('------------------------------------------------------------------\n');
disp(G_Z_de);
fprintf('\n');

fprintf('Transfer Function: psi(s) / delta_r(s)  (Yaw Heading / Rudder)\n');
fprintf('---------------------------------------------------------------\n');
disp(G_psi_dr);
fprintf('\n');

%% Step response analysis
fprintf('\nComputing step responses...\n');

% Time vector for step response
t_step = linspace(0, 80, 2000);

% Compute step responses
[y_X_XT, t_X_XT] = step(G_X_XT, t_step);
[y_Y_dr, t_Y_dr] = step(G_Y_dr, t_step);
[y_Z_de, t_Z_de] = step(G_Z_de, t_step);
[y_psi_dr, t_psi_dr] = step(G_psi_dr, t_step);

%% Plot transfer function responses
figure('Position', [100, 100, 1200, 800]);

subplot(2,2,1)
plot(t_X_XT, y_X_XT, 'LineWidth', 1.5, 'Color', [0 0.447 0.741]);
grid on;
xlabel('Time (s)', 'FontSize', 11);
ylabel('X (m)', 'FontSize', 11);
title('Step Response: X / XT (Surge Position / Thrust)', 'FontSize', 11, 'FontWeight', 'bold');

subplot(2,2,2)
plot(t_Y_dr, y_Y_dr, 'LineWidth', 1.5, 'Color', [0.85 0.325 0.098]);
grid on;
xlabel('Time (s)', 'FontSize', 11);
ylabel('Y (m)', 'FontSize', 11);
title('Step Response: Y / \delta_r (Sway Position / Rudder)', 'FontSize', 11, 'FontWeight', 'bold');

subplot(2,2,3)
plot(t_Z_de, y_Z_de, 'LineWidth', 1.5, 'Color', [0.929 0.694 0.125]);
grid on;
xlabel('Time (s)', 'FontSize', 11);
ylabel('Z (m)', 'FontSize', 11);
title('Step Response: Z / \delta_e (Heave Position / Elevator)', 'FontSize', 11, 'FontWeight', 'bold');

subplot(2,2,4)
plot(t_psi_dr, y_psi_dr, 'LineWidth', 1.5, 'Color', [0.494 0.184 0.556]);
grid on;
xlabel('Time (s)', 'FontSize', 11);
ylabel('\psi (rad)', 'FontSize', 11);
title('Step Response: \psi / \delta_r (Yaw Heading / Rudder)', 'FontSize', 11, 'FontWeight', 'bold');

sgtitle('Open-Loop Transfer Function Step Responses', 'FontSize', 13, 'FontWeight', 'bold');

% Save figure
saveas(gcf, 'Updated_OpenLoop_TF_Responses.png');
fprintf('Saved: Updated_OpenLoop_TF_Responses.png\n');

%% Plot pole-zero map
figure('Position', [100, 100, 800, 600]);
pzmap(sys);
grid on;
title('Pole-Zero Map of 4DOF REMUS-Based AUV Linear Model', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Real Axis', 'FontSize', 11);
ylabel('Imaginary Axis', 'FontSize', 11);

% Save figure
saveas(gcf, 'Updated_Pole_Zero_Map.png');
fprintf('Saved: Updated_Pole_Zero_Map.png\n');

%% Display poles and zeros
fprintf('\n========== STABILITY ANALYSIS ==========\n');
fprintf('System Poles (Eigenvalues):\n');
poles = eig(A);
for i = 1:length(poles)
    fprintf('  p%d = %.4f', i, real(poles(i)));
    if abs(imag(poles(i))) > 1e-10
        fprintf(' + %.4fj\n', imag(poles(i)));
    else
        fprintf('\n');
    end
end

% Check stability
all_stable = all(real(poles) < 0);
if all_stable
    fprintf('\nAll poles have negative real parts => STABLE\n');
else
    fprintf('\nWarning: Some poles have non-negative real parts => UNSTABLE\n');
end

fprintf('========================================\n\n');
