% Check_Trim_4DOF.m
% Verify that trim values are true equilibrium points
% 
% This script simulates the nonlinear 4DOF REMUS-based AUV model
% with NO perturbation to the trim input. If the trim values are correct,
% the velocity states should remain constant (near zero perturbations).

clear; clc; close all;

%% Load operating point and trim input
[~, ~, ~, ~, x0, U0] = auv_4dof_linear_model();

%% Simulation parameters
t_sim = 80;  % simulation time (seconds)
t = linspace(0, t_sim, 2000);

%% Trim input (no perturbation)
dU = [0; 0; 0];
U_actual = U0 + dU;

%% Nonlinear simulation with trim input
odefun = @(t, x) auv_4dof_nonlinear(t, x, U_actual);
[t_nonlin, x_nonlin] = ode45(odefun, [t(1) t(end)], x0);

% Interpolate to uniform time grid
x_nonlin_interp = interp1(t_nonlin, x_nonlin, t);

%% Convert to perturbation form
x_nonlin_pert = x_nonlin_interp - x0.';

%% Plot velocity perturbations
figure('Position', [100, 100, 900, 600]);

subplot(4,1,1)
plot(t, x_nonlin_pert(:,5), 'LineWidth', 1.5, 'Color', [0 0.447 0.741]);
grid on;
ylabel('\Delta u (m/s)', 'FontSize', 11);
title('Trim Check: Velocity Perturbations (Should Stay Near Zero)', 'FontSize', 12, 'FontWeight', 'bold');
legend('Δu', 'FontSize', 10);

subplot(4,1,2)
plot(t, x_nonlin_pert(:,6), 'LineWidth', 1.5, 'Color', [0.85 0.325 0.098]);
grid on;
ylabel('\Delta v (m/s)', 'FontSize', 11);
legend('Δv', 'FontSize', 10);

subplot(4,1,3)
plot(t, x_nonlin_pert(:,7), 'LineWidth', 1.5, 'Color', [0.929 0.694 0.125]);
grid on;
ylabel('\Delta w (m/s)', 'FontSize', 11);
legend('Δw', 'FontSize', 10);

subplot(4,1,4)
plot(t, x_nonlin_pert(:,8), 'LineWidth', 1.5, 'Color', [0.494 0.184 0.556]);
grid on;
ylabel('\Delta r (rad/s)', 'FontSize', 11);
xlabel('Time (s)', 'FontSize', 11);
legend('Δr', 'FontSize', 10);

%% Print statistics
fprintf('\n========== TRIM CHECK RESULTS ==========\n');
fprintf('If trim values are correct, perturbations should remain close to zero.\n\n');
fprintf('Velocity State Perturbations (max magnitude):\n');
fprintf('  Δu: %.4e m/s\n', max(abs(x_nonlin_pert(:,5))));
fprintf('  Δv: %.4e m/s\n', max(abs(x_nonlin_pert(:,6))));
fprintf('  Δw: %.4e m/s\n', max(abs(x_nonlin_pert(:,7))));
fprintf('  Δr: %.4e rad/s\n', max(abs(x_nonlin_pert(:,8))));
fprintf('\nIf all values are < 0.01, trim is likely correct.\n');
fprintf('========================================\n\n');

%% Save figure
saveas(gcf, 'Nonlinear_Trim_Check.png');
fprintf('Figure saved as: Nonlinear_Trim_Check.png\n');
