% PD_and_PID_Tuner_StepInfo.m
% Show step response information for the PD and PID controllers.

clc
clear
close all

%% Define transfer function
s = tf('s');
G = ((s+4.6919)*(s+0.0883)*(s+0.0013)) / ...
    (s^2*(s+0.0791)*(s+0.4354)*(s+4.7275));

%% PD controller
Kp_PD = 10;
Ki_PD = 0;
Kd_PD = 200;

C_PD = pid(Kp_PD, Ki_PD, Kd_PD);
T_PD = feedback(C_PD*G, 1);

stepinfo(T_PD)

%% PID controller
[C_PID, ~] = pidtune(G, 'PID');
T_PID = feedback(C_PID*G, 1);

stepinfo(T_PID)
