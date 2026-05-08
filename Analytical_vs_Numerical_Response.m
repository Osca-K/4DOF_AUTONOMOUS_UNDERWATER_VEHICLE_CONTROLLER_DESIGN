clear; clc; close all;

%% Define transfer function
s = tf('s');
TF3 = -0.5842/(s^2 + 0.8665*s);

%% Time vector
t = linspace(0, 20, 1000);

%% Numerical step response
[y_num, t_num] = step(TF3, t);

%% Analytical response
z_analytical = 0.7781 - 0.6742*t - 0.7781*exp(-0.8665*t);

%% Plot comparison
figure;
plot(t_num, y_num, 'LineWidth', 1.5); hold on;
plot(t, z_analytical, '--', 'LineWidth', 1.5);
grid on;

xlabel('Time (s)', 'FontName', 'Times New Roman');
ylabel('\Delta Z (m)', 'FontName', 'Times New Roman');
title('Analytical and Numerical Step Response of Heave/Depth Transfer Function', ...
      'FontName', 'Times New Roman', 'FontWeight', 'normal');

legend('Numerical MATLAB response', 'Analytical response', 'Location', 'best');

set(findall(gcf,'-property','FontName'), 'FontName', 'Times New Roman');

if ~exist('Results','dir')
    mkdir('Results');
end

saveas(gcf, fullfile('Results','Analytical_vs_Numerical_Heave_Response.png'));