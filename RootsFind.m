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

%%CPLOT ROOT LOCUS 
figure
rlocus(TF1)
grid on
title('Root Locus of Surge Position Response: X(s)/X_T(s)', ...
      'FontName', 'Times New Roman', 'FontWeight', 'normal')        
set(findall(gcf,'-property','FontName'), 'FontName', 'Times New Roman')
saveas(gcf, fullfile('Results','RootLocus_Surge_X_XT.png'))

figure
rlocus(TF2)
grid on
title('Root Locus of Sway Position Response: Y(s)/\delta_r(s)', ...
      'FontName', 'Times New Roman', 'FontWeight', 'normal')
set(findall(gcf,'-property','FontName'), 'FontName', 'Times New Roman')
saveas(gcf, fullfile('Results','RootLocus_Sway_Y_delta_r.png'))

figure
rlocus(TF3)
grid on
title('Root Locus of Heave Position Response: Z(s)/\delta_e(s)', ...
      'FontName', 'Times New Roman', 'FontWeight', 'normal')
set(findall(gcf,'-property','FontName'), 'FontName', 'Times New Roman')
saveas(gcf, fullfile('Results','RootLocus_Heave_Z_delta_e.png'))    

figure
rlocus(TF4)
grid on
title('Root Locus of Yaw Heading Response: \psi(s)/\delta_r(s)', ...
      'FontName', 'Times New Roman', 'FontWeight', 'normal')
set(findall(gcf,'-property','FontName'), 'FontName', 'Times New Roman')
saveas(gcf, fullfile('Results','RootLocus_Yaw_Psi_delta_r.png'))



