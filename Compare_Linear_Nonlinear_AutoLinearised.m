
clear; clc; close all;

%% Load numerical linear model
modelData = load('AUV_4DOF_Linearised_Model.mat', 'A', 'B', 'C', 'D', 'x_trim', 'U_trim');
A = modelData.A;
B = modelData.B;
C = modelData.C;
D = modelData.D;


x_trim = modelData.x_trim;
U_trim = modelData.U_trim;

sys_lin = ss(A, B, C, D);

%% Time for Simullation
Tfinal = 20;
t = linspace(0, Tfinal, 1000);

%% Small input perturbation
%  perturbation very small so the linear model remains valid.
dU = [0;
      0.0001;
      0];

%% Linear perturbation simulation
dx0 = zeros(8, 1);
u_lin = repmat(dU.', numel(t), 1);

[~, ~, dx_lin] = lsim(sys_lin, u_lin, t, dx0);

%% Nonlinear simulation
U_actual = U_trim + dU;
odefun = @(t, x) auv_4dof_nonlinear(t, x, U_actual);
[t_nonlin, x_nonlin] = ode45(odefun, [t(1) t(end)], x_trim);

x_nonlin_interp = interp1(t_nonlin, x_nonlin, t);
dx_nonlin = x_nonlin_interp - x_trim.';

%% Velocity perturbation comparison
figure('Position', [100, 100, 1000, 700]);
% Use Times New Roman for figure text and axes, and ensure titles are normal
set(gcf, 'DefaultAxesFontName', 'Times New Roman', 'DefaultTextFontName', 'Times New Roman');

subplot(4,1,1)
plot(t, dx_lin(:,5), 'LineWidth', 1.3); hold on;
plot(t, dx_nonlin(:,5), '--', 'LineWidth', 1.3);
grid on;
ylabel('\Delta u (m/s)');
title('Surge Velocity', 'FontWeight', 'normal', 'FontName', 'Times New Roman');
legend('Linear', 'Nonlinear');

subplot(4,1,2)
plot(t, dx_lin(:,6), 'LineWidth', 1.3); hold on;
plot(t, dx_nonlin(:,6), '--', 'LineWidth', 1.3);
grid on;
ylabel('\Delta v (m/s)');
title('Sway Velocity', 'FontWeight', 'normal', 'FontName', 'Times New Roman');
ylabel('\Delta v (m/s)');
legend('Linear', 'Nonlinear');

subplot(4,1,3)
plot(t, dx_lin(:,7), 'LineWidth', 1.3); hold on;
plot(t, dx_nonlin(:,7), '--', 'LineWidth', 1.3);
grid on;
ylabel('\Delta w (m/s)');
title('Heave Velocity', 'FontWeight', 'normal', 'FontName', 'Times New Roman');
ylabel('\Delta w (m/s)');
legend('Linear', 'Nonlinear');

subplot(4,1,4)
plot(t, dx_lin(:,8), 'LineWidth', 1.3); hold on;
plot(t, dx_nonlin(:,8), '--', 'LineWidth', 1.3);
grid on;
ylabel('\Delta r (rad/s)');
xlabel('Time (s)');
title('Yaw Rate', 'FontWeight', 'normal', 'FontName', 'Times New Roman');
ylabel('\Delta r (rad/s)');
xlabel('Time (s)');
legend('Linear', 'Nonlinear');

sgtitle('Linear vs Nonlinear Velocity', 'FontWeight', 'normal', 'FontName', 'Times New Roman');

if ~exist('Results','dir')
      mkdir('Results');
end
saveas(gcf, fullfile('Results','AutoLinearised_Velocity_Perturbations.png'));

if ~exist('Results','dir')
      mkdir('Results');
end
saveas(gcf, fullfile('Results','AutoLinearised_Position_Heading_Perturbations.png'));
