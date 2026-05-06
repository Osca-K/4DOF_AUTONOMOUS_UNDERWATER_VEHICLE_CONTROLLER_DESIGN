function dx = auv_4dof_nonlinear(t, x, U)
% ============================================================
% Active workflow step 1 of 7.
% Reduced 4DOF REMUS-based AUV nonlinear model
% States: x = [X; Y; Z; psi; u; v; w; r]
% Inputs: U = [XT; delta_r; delta_e]
% phi = 0, theta = 0, p = 0, q = 0
% Retained DOFs: surge, sway, heave, yaw
% ============================================================

%% States
psi  = x(4);

time = t; %#ok<NASGU>

u = x(5);
v = x(6);
w = x(7);
r = x(8);

%% Inputs
XT      = U(1);   % Propeller thrust
delta_r = U(2);   % Rudder deflection
delta_e = U(3);   % Elevator 4 stern deflection

%% Vehicle parameters
rho = 1.03e3;       % kg/m^3
Af  = 2.85e-2;      % m^2

B_force = 3.08e2;   % N
W_force = 2.99e2;   % N

m  = W_force/9.8;   % kg
Iz = 3.45;          % kg.m^2

%% Added mass and inertia coefficients
X_du = -0.93;
Y_dv = -35.5;
Z_dw = -35.5;
Y_dr =  1.93;
N_dv =  1.93;
N_dr = -4.88;

%% Hydrodynamic coefficients
X_vr =  35.5;
X_rr = -1.93;

Yvav = -1310;
Yrar =  0.632;
Y_ur =  5.22;
Y_uv = -28.6;

Zwaw = -131;
Z_uw = -28.6;

Nvav = -3.18;
Nrar = -94.0;
N_ur = -2.00;
N_uv = -24.0;

%% Actuator coefficients
Y_uudr =  9.64;
Z_uude = -9.64;
N_uudr = -6.15;

%% Surge drag coefficient

u_abs = max(abs(u), 1e-6);

Cd = 0.193*(u_abs)^(-0.14);
Xuau = -0.5*rho*Af*Cd*1.5;

%% 4DOF mass matrix
M4 = [m - X_du,        0,          0,        0;
      0,        m - Y_dv,          0,   -Y_dr;
      0,              0,    m - Z_dw,        0;
      0,          -N_dv,          0, Iz - N_dr];

%% Nonlinear force equations

% Surge force
F1 = Xuau*abs(u)*u ...
   + (X_vr + m)*v*r ...
   + X_rr*r^2 ...
   + XT;

% Sway force
F2 = Yvav*abs(v)*v ...
   + Yrar*abs(r)*r ...
   + (Y_ur - m)*u*r ...
   + Y_uv*u*v ...
   + Y_uudr*u^2*delta_r;

% Heave force
F3 = (W_force - B_force) ...
   + Zwaw*abs(w)*w ...
   + Z_uw*u*w ...
   + Z_uude*u^2*delta_e;

% Yaw moment
F6 = Nvav*abs(v)*v ...
   + Nrar*abs(r)*r ...
   + N_ur*u*r ...
   + N_uv*u*v ...
   + N_uudr*u^2*delta_r;

F4 = [F1; F2; F3; F6];

%% Solve accelerations
nudot = M4 \ F4;

udot = nudot(1);
vdot = nudot(2);
wdot = nudot(3);
rdot = nudot(4);

%% Kinematics
Xdot   = u*cos(psi) - v*sin(psi);
Ydot   = u*sin(psi) + v*cos(psi);
Zdot   = w;
psidot = r;

%% State derivative
dx = [Xdot;
      Ydot;
      Zdot;
      psidot;
      udot;
      vdot;
      wdot;
      rdot];

end