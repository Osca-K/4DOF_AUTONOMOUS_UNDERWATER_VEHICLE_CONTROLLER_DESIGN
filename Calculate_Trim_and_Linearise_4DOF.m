% Calculate_Trim_and_Linearise_4DOF.m
% Compute the trim condition and numerical linear model for the 4DOF AUV.
%% Trim conditions
%% Unknown trim variables
%% Solve the trim numerically
%% Check the trim residual using the nonlinear model
%% Print the trim solution
%% Finite-difference linearisation
%% Output matrices
% Calculate_Trim_and_Linearise_4DOF.m
% Active workflow step 2 of 7.
% Numerically computes the trim point and finite-difference linearisation
% directly from the nonlinear 4DOF REMUS-based AUV model.

clear; clc; close all;

%% Fixed mission conditions
X0 = 0;
Y0 = 0;
Z0 = -20;
psi0 = 0;
u0 = 2;
w0 = 0;
r0 = 0.08;

%% Unknown trim variables to solve for
% z = [v0; XT0; delta_r0; delta_e0]
z_guess = [-0.03; 15; 0.02; -0.23];

%% Solve trim numerically using a local Newton method
trimResidual = @(z) localTrimResidual(z, X0, Y0, Z0, psi0, u0, w0, r0);

[z_trim, exitflag, output] = solveTrimNumerically(trimResidual, z_guess);

v0_trim = z_trim(1);
XT0_trim = z_trim(2);
delta_r0_trim = z_trim(3);
delta_e0_trim = z_trim(4);

x_trim = [X0;
          Y0;
          Z0;
          psi0;
          u0;
          v0_trim;
          w0;
          r0];

U_trim = [XT0_trim;
          delta_r0_trim;
          delta_e0_trim];

%% Verify trim residual directly from nonlinear model
dx_trim = auv_4dof_nonlinear(0, x_trim, U_trim);
trimResidualCheck = dx_trim(5:8);

%% Print trim solution
fprintf('\n========== NUMERICAL TRIM SOLUTION ==========\n');
fprintf('Solver exitflag: %d\n', exitflag);
fprintf('Solver iterations: %d\n', output.iterations);
fprintf('Solver function evaluations: %d\n\n', output.funcCount);

fprintf('Trim state and input values:\n');
fprintf('  v0        = %.8f m/s\n', v0_trim);
fprintf('  XT0       = %.8f N\n', XT0_trim);
fprintf('  delta_r0  = %.8f rad\n', delta_r0_trim);
fprintf('  delta_e0  = %.8f rad\n\n', delta_e0_trim);

fprintf('dx_trim = auv_4dof_nonlinear(0, x_trim, U_trim):\n');
disp(dx_trim);

fprintf('Trim residual dx_trim(5:8):\n');
disp(trimResidualCheck);

%% Finite-difference linearisation
h = 1e-6;
stateDim = numel(x_trim);
inputDim = numel(U_trim);

f0 = auv_4dof_nonlinear(0, x_trim, U_trim);
A = zeros(stateDim, stateDim);
B = zeros(stateDim, inputDim);

% A matrix: partial derivatives w.r.t. states
for i = 1:stateDim
    ei = zeros(stateDim, 1);
    ei(i) = 1;

    f_plus = auv_4dof_nonlinear(0, x_trim + h * ei, U_trim);
    f_minus = auv_4dof_nonlinear(0, x_trim - h * ei, U_trim);

    A(:, i) = (f_plus - f_minus) / (2 * h);
end

% B matrix: partial derivatives w.r.t. inputs
for j = 1:inputDim
    ej = zeros(inputDim, 1);
    ej(j) = 1;

    f_plus = auv_4dof_nonlinear(0, x_trim, U_trim + h * ej);
    f_minus = auv_4dof_nonlinear(0, x_trim, U_trim - h * ej);

    B(:, j) = (f_plus - f_minus) / (2 * h);
end

%% Output matrices
C = [1 0 0 0 0 0 0 0;
     0 1 0 0 0 0 0 0;
     0 0 1 0 0 0 0 0;
     0 0 0 1 0 0 0 0];
D = zeros(4, 3);

%% Save numerical trim and linear model
save('AUV_4DOF_Linearised_Model.mat', 'A', 'B', 'C', 'D', 'x_trim', 'U_trim');

fprintf('\nSaved numerical model to AUV_4DOF_Linearised_Model.mat\n');
fprintf('A, B, C, D, x_trim, U_trim have been stored.\n');

%% Local function: trim residual
function res = localTrimResidual(z, X0, Y0, Z0, psi0, u0, w0, r0)
    v0 = z(1);
    XT0 = z(2);
    delta_r0 = z(3);
    delta_e0 = z(4);

    x_trim_local = [X0;
                    Y0;
                    Z0;
                    psi0;
                    u0;
                    v0;
                    w0;
                    r0];

    U_trim_local = [XT0;
                    delta_r0;
                    delta_e0];

    dx = auv_4dof_nonlinear(0, x_trim_local, U_trim_local);
    res = dx(5:8);
end

%% Local function: toolbox-free Newton trim solver
function [z, exitflag, output] = solveTrimNumerically(residualFcn, z0)
    z = z0(:);
    maxIter = 50;
    tolRes = 1e-10;
    tolStep = 1e-10;
    h = 1e-6;
    funcCount = 0;
    exitflag = 0;

    res = residualFcn(z);
    funcCount = funcCount + 1;
    resNorm = norm(res, 2);

    for iter = 1:maxIter
        if resNorm < tolRes
            exitflag = 1;
            break;
        end

        J = zeros(numel(res), numel(z));
        for k = 1:numel(z)
            e = zeros(numel(z), 1);
            e(k) = 1;

            resPlus = residualFcn(z + h * e);
            resMinus = residualFcn(z - h * e);
            funcCount = funcCount + 2;
            J(:, k) = (resPlus - resMinus) / (2 * h);
        end

        step = -J \ res;
        if any(~isfinite(step))
            step = -pinv(J) * res;
        end

        alpha = 1;
        accepted = false;
        for ls = 1:12
            zCandidate = z + alpha * step;
            resCandidate = residualFcn(zCandidate);
            funcCount = funcCount + 1;
            if norm(resCandidate, 2) < resNorm
                z = zCandidate;
                res = resCandidate;
                resNorm = norm(res, 2);
                accepted = true;
                break;
            end
            alpha = alpha / 2;
        end

        if ~accepted
            z = z + step;
            res = residualFcn(z);
            funcCount = funcCount + 1;
            resNorm = norm(res, 2);
        end

        if norm(alpha * step, 2) < tolStep
            exitflag = 1;
            break;
        end
    end

    output.iterations = iter;
    output.funcCount = funcCount;
    output.residualNorm = resNorm;
end
