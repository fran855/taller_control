close all
clc
clear all

aux1 = 0;
aux2 = 0;

s = tf('s');

load('identificacion_con_imu_5_grados.mat');
pulso = out.d1;
angulo = out.d2;
tiempo = out.tout;

delay = 1209;
angulo = angulo(delay:length(angulo));
tiempo = tiempo(delay:length(tiempo)) - tiempo(delay);

a = 450;
b = 40;
c = 450;

lb = 0.18;
%lp = 0.21;
lp = 0.13;
mp = 0.05;
g = 9.8;
gamma = 0.09;


figure();
for alpha = 0
    for aux2 = 0
        Gservo = tf(a+aux2, [1 b+aux1 c+aux2]);
        Gpendulo = tf([lb/lp 0 0], [1 gamma/mp g/lp]);
        Gtotal = Gpendulo * Gservo;
        opt = stepDataOptions('StepAmplitude', 30);
        [y1, t1] = step(Gtotal, opt);
        plot(t1, y1);
        hold on;
    end
end

xlim([0 4]);
plot(tiempo, angulo+2.2, 'linewidth', 2);
title(['Gamma = ', num2str(gamma)]);


