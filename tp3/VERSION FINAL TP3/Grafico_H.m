clc;
close all;
clear all;

colors = lines(4);

data = load('Controlador_con_h_perturbacion.mat');

simulacion = load('Simulacion_perturbacion_h.mat');
simulacion_2 = load('Simulacion_perturbacion_h_2.mat');

figure();
plot(data.out.tout-data.out.tout(1240), data.out.theta, 'linewidth', 2, 'Color', colors(1,:));
hold on;
plot(simulacion.out.tout-simulacion.out.tout(370), simulacion.out.theta, 'linewidth', 2, 'Color', colors(2,:));
hold on;
plot(simulacion_2.out.tout-simulacion_2.out.tout(370), 5*simulacion_2.out.perturbacion, 'linewidth', 2, 'Color', colors(3,:));
xlim([0 4]);
title('Perturbación controlador con H', 'FontSize', 24, 'FontName', 'Verdana');
legend('\theta medido', '\theta simulado');
grid on;

figure();
subplot(2, 1, 1);
plot(data.out.tout-data.out.tout(1240), data.out.theta, 'linewidth', 2, 'Color', colors(1,:));
xlim([0 4]);
title('Perturbación controlador con H', 'FontSize', 24, 'FontName', 'Verdana');
legend('\theta medido');
grid on;
subplot(2, 1, 2);
plot(simulacion.out.tout-simulacion.out.tout(380), simulacion.out.theta, 'linewidth', 2, 'Color', colors(2,:));
xlim([0 4]);
title('Perturbación controlador con H', 'FontSize', 24, 'FontName', 'Verdana');
legend('\theta simulado');
grid on;