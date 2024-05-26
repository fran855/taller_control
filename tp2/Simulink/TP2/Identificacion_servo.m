close all
clc
clear all

load('identificacion_con_imu.mat');
load('identificacion_angulo.mat');
load('identificacion_tiempo.mat');


N = length(angulo);

angulo_modelado_1 = angulo(31:N) - 91; 
tiempo_modelado_1 = tiempo(31:N) - tiempo(31);
angulo_modelado_2 = angulo(307:N) - 91;
tiempo_modelado_2 = tiempo(307:N) - tiempo(307);
angulo_modelado_3 = angulo(607:N) - 91;
tiempo_modelado_3 = tiempo(607:N) - tiempo(607);

a = 250;
b = 25;
c = a;

P = tf(a, [1 b c]);

figure();
plot(tiempo_modelado_1, angulo_modelado_1);
hold on;
opt = stepDataOptions('StepAmplitude', 30);
step(P, opt);
title(['Ajuste Gservo a = ', num2str(a), '  b = ', num2str(b), '  c = ', num2str(c)], 'Fontsize', 24);
xlabel('Time', 'FontSize', 24);
ylabel('Amplitude', 'FontSize', 24);
ax = gca;
ax.FontSize = 14;
xlim([0 0.4]);

figure();
plot(tiempo_modelado_2, angulo_modelado_2);
hold on;
opt = stepDataOptions('InputOffset', 30, 'StepAmplitude', -30);
step(P, opt);
title(['Ajuste Gservo a = ', num2str(a), '  b = ', num2str(b), '  c = ', num2str(c)], 'Fontsize', 24);
xlabel('Time', 'FontSize', 24);
ylabel('Amplitude', 'FontSize', 24);
ax = gca;
ax.FontSize = 14;
xlim([0 0.4]);

figure();
plot(tiempo_modelado_3, angulo_modelado_3);
hold on;
opt = stepDataOptions('StepAmplitude', 30);
step(P, opt);
title(['Ajuste Gservo a = ', num2str(a), '  b = ', num2str(b), '  c = ', num2str(c)], 'Fontsize', 24);
xlabel('Time', 'FontSize', 24);
ylabel('Amplitude', 'FontSize', 24);
ax = gca;
ax.FontSize = 14;
xlim([0 0.4]);

%{
figure();
plot(tiempo_modelado(578:1759)-5.76, angulo_modelado(578:1759));
for a = 4500:500:4500
    hold on;
    c = a;
    
    
    i = i+1;
end

title(['a = 4500 b = 400 c = 4500']);
%}