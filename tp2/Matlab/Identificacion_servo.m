close all
clc
clear all

load('identificacion_con_imu.mat');

angulo = out.d1;
tiempo = out.tout;

N = length(angulo);

angulo_modelado_1 = angulo(806:N) - 91; 
tiempo_modelado_1 = tiempo(806:N) - tiempo(806);
angulo_modelado_2 = angulo(406:N) - 91;
tiempo_modelado_2 = tiempo(406:N) - tiempo(406);
angulo_modelado_3 = angulo(2404:N) - 91;
tiempo_modelado_3 = tiempo(2404:N) - tiempo(2404);

a = 450;
b = 40;
c = 450;

P = tf(a, [1 b c]);

figure();
plot(tiempo_modelado_1, angulo_modelado_1);
hold on;
opt = stepDataOptions('StepAmplitude', 30);
step(P, opt);
title(['a = ', num2str(a), '  b = ', num2str(b), '  c = ', num2str(c)]);
xlim([0 0.4]);

figure();
plot(tiempo_modelado_2, angulo_modelado_2);
hold on;
opt = stepDataOptions('InputOffset', 30, 'StepAmplitude', -30);
step(P, opt);
title(['a = ', num2str(a), '  b = ', num2str(b), '  c = ', num2str(c)]);
xlim([0 0.4]);

figure();
plot(tiempo_modelado_3, angulo_modelado_3);
hold on;
opt = stepDataOptions('StepAmplitude', 30);
step(P, opt);
title(['a = ', num2str(a), '  b = ', num2str(b), '  c = ', num2str(c)]);
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