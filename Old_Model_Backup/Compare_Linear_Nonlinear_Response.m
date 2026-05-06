clear; clc; close all;

%% Load linear model
[A, B, C, D, x0, U0] = auv_4dof_linear_model();

sys_lin = ss(A, B, C, D);

%% Simulation time
t = linspace(0, 80, 2000);

%% Define input perturbation for linear model
% Example: small rudder perturbation
dXT      = 0;        % N
ddelta_r = 0.001;    % rad
ddelta_e = 0;        % rad

dU = [dXT;
      ddelta_r;
      ddelta_e];

%% Linear simulation
u_lin = repmat(dU.', length(t), 1);

% Initial perturbation state is zero
dx0 = zeros(8,1);

[y_lin, t_lin, x_lin_pert] = lsim(sys_lin, u_lin, t, dx0);

% Convert perturbation states back to actual states
x_lin_actual = x_lin_pert + x0.';

%% Nonlinear simulation
% Actual input = trim input + perturbation
U_actual = U0 + dU;

odefun = @(t, x) auv_4dof_nonlinear(t, x, U_actual);

[t_nonlin, x_nonlin] = ode45(odefun, [t(1) t(end)], x0);

%% Interpolate nonlinear response to same time vector
x_nonlin_interp = interp1(t_nonlin, x_nonlin, t);

%% Convert nonlinear response to perturbation form
x_nonlin_pert = x_nonlin_interp - x0.';

%% Plot position and heading comparison (perturbations)
figure;

subplot(4,1,1)
plot(t, x_lin_pert(:,1), 'LineWidth', 1.2); hold on;
plot(t, x_nonlin_pert(:,1), '--', 'LineWidth', 1.2);
grid on;
ylabel('\DeltaX (m)');
legend('Linear', 'Nonlinear');

subplot(4,1,2)
plot(t, x_lin_pert(:,2), 'LineWidth', 1.2); hold on;
plot(t, x_nonlin_pert(:,2), '--', 'LineWidth', 1.2);
grid on;
ylabel('\DeltaY (m)');

subplot(4,1,3)
plot(t, x_lin_pert(:,3), 'LineWidth', 1.2); hold on;
plot(t, x_nonlin_pert(:,3), '--', 'LineWidth', 1.2);
grid on;
ylabel('\DeltaZ (m)');

subplot(4,1,4)
plot(t, x_lin_pert(:,4), 'LineWidth', 1.2); hold on;
plot(t, x_nonlin_pert(:,4), '--', 'LineWidth', 1.2);
grid on;
ylabel('\Delta\psi (rad)');
xlabel('Time (s)');

sgtitle('Linear vs Nonlinear Position and Heading Perturbations');

%% Plot velocity comparison
figure;

subplot(4,1,1)
plot(t, x_lin_pert(:,5), 'LineWidth', 1.2); hold on;
plot(t, x_nonlin_pert(:,5), '--', 'LineWidth', 1.2);
grid on;
ylabel('\Deltau (m/s)');
legend('Linear', 'Nonlinear');

subplot(4,1,2)
plot(t, x_lin_pert(:,6), 'LineWidth', 1.2); hold on;
plot(t, x_nonlin_pert(:,6), '--', 'LineWidth', 1.2);
grid on;
ylabel('\Deltav (m/s)');

subplot(4,1,3)
plot(t, x_lin_pert(:,7), 'LineWidth', 1.2); hold on;
plot(t, x_nonlin_pert(:,7), '--', 'LineWidth', 1.2);
grid on;
ylabel('\Deltaw (m/s)');

subplot(4,1,4)
plot(t, x_lin_pert(:,8), 'LineWidth', 1.2); hold on;
plot(t, x_nonlin_pert(:,8), '--', 'LineWidth', 1.2);
grid on;
ylabel('\Deltar (rad/s)');
xlabel('Time (s)');

sgtitle('Linear vs Nonlinear Velocity Perturbations');

%% Save figures
saveas(1, 'Linear_vs_Nonlinear_Position_Heading_Perturbations.png');
saveas(2, 'Linear_vs_Nonlinear_Velocities.png');clear; clc; close all;

%% Load linear model
[A, B, C, D, x0, U0] = auv_4dof_linear_model();

sys_lin = ss(A, B, C, D);

%% Simulation time
t = linspace(0, 80, 2000);

%% Define input perturbation for linear model
% Example: small rudder perturbation
dXT      = 0;        % N
ddelta_r = 0.001;    % rad
ddelta_e = 0;        % rad

dU = [dXT;
      ddelta_r;
      ddelta_e];

%% Linear simulation
u_lin = repmat(dU.', length(t), 1);

% Initial perturbation state is zero
dx0 = zeros(8,1);

[y_lin, t_lin, x_lin_pert] = lsim(sys_lin, u_lin, t, dx0);

% Convert perturbation states back to actual states
x_lin_actual = x_lin_pert + x0.';

%% Nonlinear simulation
% Actual input = trim input + perturbation
U_actual = U0 + dU;

odefun = @(t, x) auv_4dof_nonlinear(t, x, U_actual);

[t_nonlin, x_nonlin] = ode45(odefun, [t(1) t(end)], x0);

%% Interpolate nonlinear response to same time vector
x_nonlin_interp = interp1(t_nonlin, x_nonlin, t);

%% Convert nonlinear response to perturbation form
x_nonlin_pert = x_nonlin_interp - x0.';

%% Plot position and heading comparison (perturbations)
figure;

subplot(4,1,1)
plot(t, x_lin_pert(:,1), 'LineWidth', 1.2); hold on;
plot(t, x_nonlin_pert(:,1), '--', 'LineWidth', 1.2);
grid on;
ylabel('\DeltaX (m)');
legend('Linear', 'Nonlinear');

subplot(4,1,2)
plot(t, x_lin_pert(:,2), 'LineWidth', 1.2); hold on;
plot(t, x_nonlin_pert(:,2), '--', 'LineWidth', 1.2);
grid on;
ylabel('\DeltaY (m)');

subplot(4,1,3)
plot(t, x_lin_pert(:,3), 'LineWidth', 1.2); hold on;
plot(t, x_nonlin_pert(:,3), '--', 'LineWidth', 1.2);
grid on;
ylabel('\DeltaZ (m)');

subplot(4,1,4)
plot(t, x_lin_pert(:,4), 'LineWidth', 1.2); hold on;
plot(t, x_nonlin_pert(:,4), '--', 'LineWidth', 1.2);
grid on;
ylabel('\Delta\psi (rad)');
xlabel('Time (s)');

sgtitle('Linear vs Nonlinear Position and Heading Perturbations');

%% Plot velocity comparison
figure;

subplot(4,1,1)
plot(t, x_lin_pert(:,5), 'LineWidth', 1.2); hold on;
plot(t, x_nonlin_pert(:,5), '--', 'LineWidth', 1.2);
grid on;
ylabel('\Deltau (m/s)');
legend('Linear', 'Nonlinear');

subplot(4,1,2)
plot(t, x_lin_pert(:,6), 'LineWidth', 1.2); hold on;
plot(t, x_nonlin_pert(:,6), '--', 'LineWidth', 1.2);
grid on;
ylabel('\Deltav (m/s)');

subplot(4,1,3)
plot(t, x_lin_pert(:,7), 'LineWidth', 1.2); hold on;
plot(t, x_nonlin_pert(:,7), '--', 'LineWidth', 1.2);
grid on;
ylabel('\Deltaw (m/s)');

subplot(4,1,4)
plot(t, x_lin_pert(:,8), 'LineWidth', 1.2); hold on;
plot(t, x_nonlin_pert(:,8), '--', 'LineWidth', 1.2);
grid on;
ylabel('\Deltar (rad/s)');
xlabel('Time (s)');

sgtitle('Linear vs Nonlinear Velocity Perturbations');

%% Save figures
saveas(1, 'Linear_vs_Nonlinear_Position_Heading_Perturbations.png');
saveas(2, 'Linear_vs_Nonlinear_Velocity_Perturbations.png');