clear; clc; close all;

%% Load 4DOF linear model
[A, B, C, D, x_trim, U_trim] = auv_4dof_linear_model();

%% State-space system
sys = ss(A, B, C, D);

%% Transfer functions
TF = tf(sys);

TF1 = TF(1,1);   % X(s) / X_T(s)
TF2 = TF(2,2);   % Y(s) / S_r(s)
TF3 = TF(3,3);   % Z(s) / S_e(s)
TF4 = TF(4,2);   % yaw(s) / S_r(s)


%%ROOTS OF THE TRANSFER FUNCTIONS

%%Poles and Zeros of TF1

P1=pole(TF1)
Z1=zero(TF1)

%%Poles and Zeros of TF2
P2=pole(TF2)
Z2=zero(TF2)
%%Poles and Zeros of TF3
P3=pole(TF3)
Z3=zero(TF3)
%%Poles and Zeros of TF4
P4=pole(TF4)
Z4=zero(TF4)

