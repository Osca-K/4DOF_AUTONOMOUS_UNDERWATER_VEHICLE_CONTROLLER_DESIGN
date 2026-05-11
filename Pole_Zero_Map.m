% PZMAP.m
% Plot pole-zero 

clear; clc; close all;

%% Load 4DOF linear model
[A, B, C, D, x_trim, U_trim] = auv_4dof_linear_model();

%% Create state-space system
sys = ss(A, B, C, D);

%% Create transfer functions
TF = tf(sys);

% Extract individual transfer functions of interest
TF1 = TF(1,1);   % X position / thrust
TF2 = TF(2,2);   % Y position / rudder
TF3 = TF(3,3);   % Z position / elevator
TF4 = TF(4,2);   % Yaw heading / rudder

%% Create Results folder
if ~exist('Results','dir')
    mkdir('Results');
end

%% Pole-zero maps

%% Surge
figure
pzmap(TF1)
grid on
title('Pole-Zero Map of Surge Position Response: X(s)/X_T(s)', ...
      'FontName', 'Times New Roman', 'FontWeight', 'normal')
xlabel('Real Axis', 'FontName', 'Times New Roman')
ylabel('Imaginary Axis', 'FontName', 'Times New Roman')
set(findall(gcf,'-property','FontName'), 'FontName', 'Times New Roman')
saveas(gcf, fullfile('Results','PoleZero_Surge_X_XT.png'))

%% Sway
figure
pzmap(TF2)
grid on
title('Pole-Zero Map of Sway Position Response: Y(s)/\delta_r(s)', ...
      'FontName', 'Times New Roman', 'FontWeight', 'normal')
xlabel('Real Axis', 'FontName', 'Times New Roman')
ylabel('Imaginary Axis', 'FontName', 'Times New Roman')
set(findall(gcf,'-property','FontName'), 'FontName', 'Times New Roman')
saveas(gcf, fullfile('Results','PoleZero_Sway_Y_delta_r.png'))

%% Heave
figure
pzmap(TF3)
grid on
title('Pole-Zero Map of Heave Position Response: Z(s)/\delta_e(s)', ...
      'FontName', 'Times New Roman', 'FontWeight', 'normal')
xlabel('Real Axis', 'FontName', 'Times New Roman')
ylabel('Imaginary Axis', 'FontName', 'Times New Roman')
set(findall(gcf,'-property','FontName'), 'FontName', 'Times New Roman')
saveas(gcf, fullfile('Results','PoleZero_Heave_Z_delta_e.png'))

%% Yaw
figure
pzmap(TF4)
grid on
title('Pole-Zero Map of Yaw Heading Response: \psi(s)/\delta_r(s)', ...
      'FontName', 'Times New Roman', 'FontWeight', 'normal')
xlabel('Real Axis', 'FontName', 'Times New Roman')
ylabel('Imaginary Axis', 'FontName', 'Times New Roman')
set(findall(gcf,'-property','FontName'), 'FontName', 'Times New Roman')
saveas(gcf, fullfile('Results','PoleZero_Yaw_psi_delta_r.png'))


