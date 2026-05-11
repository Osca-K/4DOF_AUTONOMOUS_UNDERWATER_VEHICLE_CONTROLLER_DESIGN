% Blode_Plot.m
% Plot the Bode responses for the selected 4DOF AUV transfer functions.


clear; clc; close all;

%% Load 4DOF linear model
[A, B, C, D, x_trim, U_trim] = auv_4dof_linear_model();

%% State-space system
sys = ss(A, B, C, D);

%% Transfer functions
TF = tf(sys);

TF1 = TF(1,1);   % X position / thrust
TF2 = TF(2,2);   % Y position / rudder
TF3 = TF(3,3);   % Z position / elevator
TF4 = TF(4,2);   % yaw heading / rudder

%% Create Results folder
if ~exist('Results','dir')
    mkdir('Results');
end

%% Bode Plot of Surge
figure
bode(TF1, {0.01, 10});
grid on
title('Bode Plot of Surge', 'FontName', 'Times New Roman', 'FontWeight', 'normal')
set(findall(gcf,'-property','FontName'), 'FontName', 'Times New Roman')
saveas(gcf, fullfile('Results','Bode_Surge.png'))

%% Bode Plot of Sway
figure
bode(TF2, {0.1, 10});
grid on
title('Bode Plot of Sway', 'FontName', 'Times New Roman', 'FontWeight', 'normal')
set(findall(gcf,'-property','FontName'), 'FontName', 'Times New Roman')
saveas(gcf, fullfile('Results','Bode_Sway.png'))

%% Bode Plot of Heave
figure
bode(TF3, {0.01, 10});
grid on
title('Bode Plot of Heave', 'FontName', 'Times New Roman', 'FontWeight', 'normal')
set(findall(gcf,'-property','FontName'), 'FontName', 'Times New Roman')
saveas(gcf, fullfile('Results','Bode_Heave.png'))

%% Bode Plot of Yaw
figure
bode(TF4, {0.001, 100});
grid on
title('Bode Plot of Yaw', 'FontName', 'Times New Roman', 'FontWeight', 'normal')
set(findall(gcf,'-property','FontName'), 'FontName', 'Times New Roman')
saveas(gcf, fullfile('Results','Bode_Yaw.png'))