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

%% ============================================================
%  BODE PLOTS
%  Commented out for now
% =============================================================

% %% Bode plot options
% opts = bodeoptions;
% opts.Grid = 'on';
% opts.FreqUnits = 'rad/s';
% opts.Title.FontSize = 12;
% opts.Title.FontWeight = 'normal';
% opts.XLabel.FontSize = 11;
% opts.YLabel.FontSize = 11;
% opts.TickLabel.FontSize = 10;
% 
% freqRange = {0.01, 100};
% 
% %% Bode Plot 1: Surge
% figure('Position', [100, 100, 800, 600]);
% bodeplot(TF1, freqRange, opts);
% grid on;
% title('Bode Plot of Surge Position Response: X(s)/X_T(s)', ...
%       'FontName', 'Times New Roman', 'FontWeight', 'normal');
% set(findall(gcf,'-property','FontName'), 'FontName', 'Times New Roman');
% saveas(gcf, fullfile('Results','Bode_Surge_X_XT.png'));
% 
% %% Bode Plot 2: Sway
% figure('Position', [150, 100, 800, 600]);
% bodeplot(TF2, freqRange, opts);
% grid on;
% title('Bode Plot of Sway Position Response: Y(s)/\delta_r(s)', ...
%       'FontName', 'Times New Roman', 'FontWeight', 'normal');
% set(findall(gcf,'-property','FontName'), 'FontName', 'Times New Roman');
% saveas(gcf, fullfile('Results','Bode_Sway_Y_delta_r.png'));
% 
% %% Bode Plot 3: Heave
% figure('Position', [200, 100, 800, 600]);
% bodeplot(TF3, freqRange, opts);
% grid on;
% title('Bode Plot of Heave Position Response: Z(s)/\delta_e(s)', ...
%       'FontName', 'Times New Roman', 'FontWeight', 'normal');
% set(findall(gcf,'-property','FontName'), 'FontName', 'Times New Roman');
% saveas(gcf, fullfile('Results','Bode_Heave_Z_delta_e.png'));
% 
% %% Bode Plot 4: Yaw
% figure('Position', [250, 100, 800, 600]);
% bodeplot(TF4, freqRange, opts);
% grid on;
% title('Bode Plot of Yaw Heading Response: \psi(s)/\delta_r(s)', ...
%       'FontName', 'Times New Roman', 'FontWeight', 'normal');
% set(findall(gcf,'-property','FontName'), 'FontName', 'Times New Roman');
% saveas(gcf, fullfile('Results','Bode_Yaw_psi_delta_r.png'));


%% ============================================================
%  POLE-ZERO MAPS
% =============================================================

%% Pole-Zero Map 1: Surge
figure
pzmap(TF1)
grid on
title('Pole-Zero Map of Surge Position Response: X(s)/X_T(s)', ...
      'FontName', 'Times New Roman', 'FontWeight', 'normal')
xlabel('Real Axis', 'FontName', 'Times New Roman')
ylabel('Imaginary Axis', 'FontName', 'Times New Roman')
set(findall(gcf,'-property','FontName'), 'FontName', 'Times New Roman')
saveas(gcf, fullfile('Results','PoleZero_Surge_X_XT.png'))

%% Pole-Zero Map 2: Sway
figure
pzmap(TF2)
grid on
title('Pole-Zero Map of Sway Position Response: Y(s)/\delta_r(s)', ...
      'FontName', 'Times New Roman', 'FontWeight', 'normal')
xlabel('Real Axis', 'FontName', 'Times New Roman')
ylabel('Imaginary Axis', 'FontName', 'Times New Roman')
set(findall(gcf,'-property','FontName'), 'FontName', 'Times New Roman')
saveas(gcf, fullfile('Results','PoleZero_Sway_Y_delta_r.png'))

%% Pole-Zero Map 3: Heave
figure
pzmap(TF3)
grid on
title('Pole-Zero Map of Heave Position Response: Z(s)/\delta_e(s)', ...
      'FontName', 'Times New Roman', 'FontWeight', 'normal')
xlabel('Real Axis', 'FontName', 'Times New Roman')
ylabel('Imaginary Axis', 'FontName', 'Times New Roman')
set(findall(gcf,'-property','FontName'), 'FontName', 'Times New Roman')
saveas(gcf, fullfile('Results','PoleZero_Heave_Z_delta_e.png'))

%% Pole-Zero Map 4: Yaw
figure
pzmap(TF4)
grid on
title('Pole-Zero Map of Yaw Heading Response: \psi(s)/\delta_r(s)', ...
      'FontName', 'Times New Roman', 'FontWeight', 'normal')
xlabel('Real Axis', 'FontName', 'Times New Roman')
ylabel('Imaginary Axis', 'FontName', 'Times New Roman')
set(findall(gcf,'-property','FontName'), 'FontName', 'Times New Roman')
saveas(gcf, fullfile('Results','PoleZero_Yaw_psi_delta_r.png'))


%% Display poles and zeros in command window
fprintf('\n========== POLES AND ZEROS ==========\n');

fprintf('\nSurge: X(s)/X_T(s)\n');
disp('Poles:'); disp(pole(TF1));
disp('Zeros:'); disp(zero(TF1));

fprintf('\nSway: Y(s)/delta_r(s)\n');
disp('Poles:'); disp(pole(TF2));
disp('Zeros:'); disp(zero(TF2));

fprintf('\nHeave: Z(s)/delta_e(s)\n');
disp('Poles:'); disp(pole(TF3));
disp('Zeros:'); disp(zero(TF3));

fprintf('\nYaw: psi(s)/delta_r(s)\n');
disp('Poles:'); disp(pole(TF4));
disp('Zeros:'); disp(zero(TF4));