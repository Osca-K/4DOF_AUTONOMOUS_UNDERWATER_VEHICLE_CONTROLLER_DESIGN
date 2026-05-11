% Step_Input_Response.m
% Plot the step responses for the selected 4DOF AUV transfer functions.

clear; clc; close all;

%% Load the linear model
[A, B, C, D, x_trim, U_trim] = auv_4dof_linear_model();

%% State-space model
sys = ss(A, B, C, D);

%% Transfer functions
TF = tf(sys);

TF1 = TF(1,1);   % X(s) / X_T(s)
TF2 = TF(2,2);   % Y(s) / delta_r(s)
TF3 = TF(3,3);   % Z(s) / delta_e(s)
TF4 = TF(4,2);   % psi(s) / delta_r(s)

%% Create the Results folder
if ~exist('Results','dir')
    mkdir('Results');
end

%% Time vector
t = 0:0.01:20;

%% Surge position / thrust
figure
step(TF1, t)
grid on
title('Step Response of Surge Position: X(s)/X_T(s)', ...
      'FontName', 'Times New Roman', 'FontWeight', 'normal')
xlabel('Time (s)', 'FontName', 'Times New Roman')
ylabel('\Delta X (m)', 'FontName', 'Times New Roman')
set(findall(gcf,'-property','FontName'), 'FontName', 'Times New Roman')
saveas(gcf, fullfile('Results','Step_Surge_X_XT.png'))

%% Sway position / rudder
figure
step(TF2, t)
grid on
title('Step Response of Sway Position: Y(s)/\delta_r(s)', ...
      'FontName', 'Times New Roman', 'FontWeight', 'normal')
xlabel('Time (s)', 'FontName', 'Times New Roman')
ylabel('\Delta Y (m)', 'FontName', 'Times New Roman')
set(findall(gcf,'-property','FontName'), 'FontName', 'Times New Roman')
saveas(gcf, fullfile('Results','Step_Sway_Y_delta_r.png'))

%% Heave position / elevator
figure
step(TF3, t)
grid on
title('Step Response of Heave Position: Z(s)/\delta_e(s)', ...
      'FontName', 'Times New Roman', 'FontWeight', 'normal')
xlabel('Time (s)', 'FontName', 'Times New Roman')
ylabel('\Delta Z (m)', 'FontName', 'Times New Roman')
set(findall(gcf,'-property','FontName'), 'FontName', 'Times New Roman')
saveas(gcf, fullfile('Results','Step_Heave_Z_delta_e.png'))

%% Yaw heading / rudder
figure
step(TF4, t)
grid on
title('Step Response of Yaw Heading: \psi(s)/\delta_r(s)', ...
      'FontName', 'Times New Roman', 'FontWeight', 'normal')
xlabel('Time (s)', 'FontName', 'Times New Roman')
ylabel('\Delta \psi (rad)', 'FontName', 'Times New Roman')
set(findall(gcf,'-property','FontName'), 'FontName', 'Times New Roman')
saveas(gcf, fullfile('Results','Step_Yaw_psi_delta_r.png'))