clear; clc; close all;

%% Load 4DOF linear model
[A, B, C, D, x_trim, U_trim] = auv_4dof_linear_model();

%% Output velocity states instead of position states
C_vel = [0 0 0 0 1 0 0 0;   % u
         0 0 0 0 0 1 0 0;   % v
         0 0 0 0 0 0 1 0;   % w
         0 0 0 0 0 0 0 1];  % r

D_vel = zeros(4,3);

%% Velocity state-space system
sys_vel = ss(A, B, C_vel, D_vel);

%% Velocity transfer functions
TF_vel = tf(sys_vel);

G_u_XT = TF_vel(1,1);   % surge velocity / thrust
G_v_dr = TF_vel(2,2);   % sway velocity / rudder
G_w_de = TF_vel(3,3);   % heave velocity / elevator
G_r_dr = TF_vel(4,2);   % yaw rate / rudder

disp('G_u_XT = u(s)/X_T(s)')
G_u_XT

disp('G_v_dr = v(s)/delta_r(s)')
G_v_dr

disp('G_w_de = w(s)/delta_e(s)')
G_w_de

disp('G_r_dr = r(s)/delta_r(s)')
G_r_dr