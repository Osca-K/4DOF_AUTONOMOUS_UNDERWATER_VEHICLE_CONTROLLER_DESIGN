clc
clear
close all

s = tf('s');

G = ((s+4.6919)*(s+0.0883)*(s+0.0013)) / ...
    (s^2*(s+0.0791)*(s+0.4354)*(s+4.7275));

% Automatic tuning
[C,info] = pidtune(G,'PID');

% Closed-loop system
T = feedback(C*G,1);

% Ensure Results folder exists
if ~exist('Results','dir')
    mkdir('Results');
end

figure
step(T)
grid on
title('Closed-Loop Step Response', 'FontName', 'Times New Roman', 'FontSize', 12)
xlabel('Time (s)', 'FontName', 'Times New Roman', 'FontSize', 11)
ylabel('Amplitude', 'FontName', 'Times New Roman', 'FontSize', 11)
set(findall(gcf,'-property','FontName'), 'FontName', 'Times New Roman')
set(findall(gcf,'-property','FontSize'), 'FontSize', 10)
saveas(gcf, fullfile('Results','PID_ClosedLoop_Step.png'));

figure
rlocus(C*G)
grid on
title('Root Locus: PID Controller Design', 'FontName', 'Times New Roman', 'FontSize', 12)
xlabel('Real Axis', 'FontName', 'Times New Roman', 'FontSize', 11)
ylabel('Imaginary Axis', 'FontName', 'Times New Roman', 'FontSize', 11)
set(findall(gcf,'-property','FontName'), 'FontName', 'Times New Roman')
set(findall(gcf,'-property','FontSize'), 'FontSize', 10)
saveas(gcf, fullfile('Results','PID_RootLocus.png'));

figure
margin(C*G)
grid on
title('Bode Margins: PID*G', 'FontName', 'Times New Roman', 'FontSize', 12)
set(findall(gcf,'-property','FontName'), 'FontName', 'Times New Roman')
set(findall(gcf,'-property','FontSize'), 'FontSize', 10)
saveas(gcf, fullfile('Results','PID_Margin.png'));