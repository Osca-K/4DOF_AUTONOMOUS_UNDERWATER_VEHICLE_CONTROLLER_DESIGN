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

