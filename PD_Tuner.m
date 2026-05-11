clc
clear
close all

s = tf('s');

G = ((s+4.6919)*(s+0.0883)*(s+0.0013)) / ...
    (s^2*(s+0.0791)*(s+0.4354)*(s+4.7275));

%pidTuner(G,'PD');

%[C,info] = pidtune(G,'PD');
Kp = 10;
Ki = 0;
Kd = 200;

C = pid(Kp,Ki,Kd);
T = feedback(C*G,1);

%pole(T)
%step(T)
%rlocus(C*G)
%margin(C*G)

% Ensure Results folder exists
if ~exist('Results','dir')
    mkdir('Results');
end

figure
step(T)
grid on
% title('Closed-Loop Step Response', 'FontName', 'Times New Roman', 'FontSize', 12)
xlabel('Time (s)', 'FontName', 'Times New Roman', 'FontSize', 11)
ylabel('Amplitude', 'FontName', 'Times New Roman', 'FontSize', 11)
set(findall(gcf,'-property','FontName'), 'FontName', 'Times New Roman')
set(findall(gcf,'-property','FontSize'), 'FontSize', 10)
saveas(gcf, fullfile('Results','RootLocus2_Step.png'));

figure
rlocus(C*G)
grid on
% title('Root Locus: PD Controller Design', 'FontName', 'Times New Roman', 'FontSize', 12)
xlabel('Real Axis', 'FontName', 'Times New Roman', 'FontSize', 11)
ylabel('Imaginary Axis', 'FontName', 'Times New Roman', 'FontSize', 11)
set(findall(gcf,'-property','FontName'), 'FontName', 'Times New Roman')
set(findall(gcf,'-property','FontSize'), 'FontSize', 10)
saveas(gcf, fullfile('Results','RootLocus2_RootLocus.png'));

figure
margin(C*G)
grid on
% title('Bode Magnitude and Phase: PD*G', 'FontName', 'Times New Roman', 'FontSize', 12)
set(findall(gcf,'-property','FontName'), 'FontName', 'Times New Roman')
set(findall(gcf,'-property','FontSize'), 'FontSize', 10)
saveas(gcf, fullfile('Results','RootLocus2_Margin.png'));