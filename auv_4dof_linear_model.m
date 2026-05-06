function [A, B, C, D, x_trim, U_trim] = auv_4dof_linear_model()
% Load the numerically linearised 4DOF REMUS-based AUV model.
% Run Calculate_Trim_and_Linearise_4DOF.m first to generate the MAT file.

modelFile = 'AUV_4DOF_Linearised_Model.mat';

if ~isfile(modelFile)
    error(['Missing %s. Run Calculate_Trim_and_Linearise_4DOF.m first to ', ...
           'generate the numerical trim and linear model.'], modelFile);
end

modelData = load(modelFile, 'A', 'B', 'C', 'D', 'x_trim', 'U_trim');

A = modelData.A;
B = modelData.B;
C = modelData.C;
D = modelData.D;
x_trim = modelData.x_trim;
U_trim = modelData.U_trim;

end