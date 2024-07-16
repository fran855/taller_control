clc;
close all;
clear all;

%data = load('Controlador_sin_f.mat');
%data = load('Controlador_con_f.mat');
data = load('Controlador_con_h.mat');

%step_f = load('Controlador_con_f_step_70_110.mat');
%step_f = load('Controlador_con_f_step_80_100.mat');
%step_f2 = load('Controlador_con_f_step_70_110.mat');
%step_f3 = load('Controlador_con_f_step_60_120.mat');

%simulacion = load('Simulacion_sin_f.mat');
%simulacion = load('Simulacion_con_f.mat');

%simulacion_step = load('Simulacion_con_f_step.mat');

figure();
subplot(4,1,1);
plot(data.out.tout, data.out.phi-3, 'linewidth', 1.5);
hold on;
plot(data.out.tout, data.out.phi_est-3, 'linewidth', 1.5);
%xlim([24 60]);
xlim([0.5 30]);
xlim([7 25]);
title('Ángulo \phi');
legend('\phi', '\phi estimado');
grid on;

subplot(4,1,2);
plot(data.out.tout, data.out.theta, 'linewidth', 1.5);
hold on;
plot(data.out.tout, data.out.theta_est, 'linewidth', 1.5);
%xlim([24 60]);
xlim([0.5 30]);
xlim([7 25]);
title('Ángulo \theta');
legend('\theta', '\theta estimado');
grid on;

subplot(4,1,3);
plot(data.out.tout, data.out.dphi_est, 'linewidth', 1.5);
%xlim([24 60]);
%xlim([0.5 30]);
ylim([-320 320]);
xlim([7 25]);
title('Velocidad angular d\phi');
legend('d\phi');
grid on;

subplot(4,1,4);
plot(data.out.tout, data.out.dtheta, 'linewidth', 1.5);
hold on;
plot(data.out.tout, data.out.dtheta_est, 'linewidth', 1.5);
%xlim([24 60]);
xlim([0.5 30]);
xlim([7 25]);
title('Velocidad angular d\theta');
legend('d\theta', 'd\theta estimado');
grid on;

figure();
%plot(data.out.tout-data.out.tout(2132), data.out.theta, 'linewidth', 1.5);
plot(data.out.tout-data.out.tout(310), data.out.theta, 'linewidth', 1.5);
hold on;
%plot(data.out.tout-data.out.tout(2132), data.out.theta_est, 'linewidth', 1.5);
plot(data.out.tout-data.out.tout(310), data.out.theta_est, 'linewidth', 1.5);
xlim([0 2.5]);
title('Ángulo \theta');
legend('\theta', '\theta estimado');
grid on;

figure();
subplot(2,1,1);
%plot(data.out.tout-data.out.tout(2132), data.out.theta, 'linewidth', 2);

plot(data.out.tout-data.out.tout(310), data.out.phi, 'linewidth', 2);
hold on;
%plot(simulacion.out.theta.Time(200:400)-simulacion.out.theta.Time(200), simulacion.out.theta.Data(200:400), 'linewidth', 2);
%plot(simulacion.out.theta.Time-simulacion.out.theta.Time(175), simulacion.out.theta.Data, 'linewidth', 2);
plot(simulacion.out.phi.Time-simulacion.out.phi.Time(185), simulacion.out.phi.Data, 'linewidth', 2);
xlim([0 2.5]);
title('Ángulo \phi. Controlador con Feedforward');
legend('\phi', '\phi simulado');
grid on;

subplot(2,1,2);
plot(data.out.tout-data.out.tout(310), data.out.theta, 'linewidth', 2);
hold on;
plot(simulacion.out.theta.Time-simulacion.out.theta.Time(220), simulacion.out.theta.Data, 'linewidth', 2);
xlim([0 2.5]);
title('Ángulo \theta. Controlador con Feedforward');
legend('\theta', '\theta simulado');
grid on;

figure();
subplot(3,1,1);
plot(step_f.out.tout-step_f.out.tout(740), step_f.out.phi_ref, 'linewidth', 2);
hold on;
plot(step_f.out.tout-step_f.out.tout(740), step_f.out.phi, 'linewidth', 2);
ylim([75 115]);
xlim([0 40]);
title('Ángulo \phi seguimiento de referencia');
legend('\phi ref', '\phi');
grid on;

subplot(3,1,2);
plot(step_f2.out.tout-step_f2.out.tout(740), step_f2.out.phi_ref, 'linewidth', 2);
hold on;
plot(step_f2.out.tout-step_f2.out.tout(740), step_f2.out.phi, 'linewidth', 2);
ylim([65 125]);
xlim([0 40]);
title('Ángulo \phi seguimiento de referencia');
legend('\phi ref', '\phi');
grid on;

subplot(3,1,3);
plot(step_f3.out.tout-step_f3.out.tout(740), step_f3.out.phi_ref, 'linewidth', 2);
hold on;
plot(step_f3.out.tout-step_f3.out.tout(740), step_f3.out.phi, 'linewidth', 2);
ylim([55 135]);
xlim([0 40]);
title('Ángulo \phi seguimiento de referencia');
legend('\phi ref', '\phi');
grid on;

figure();
plot(simulacion_step.out.phi, 'linewidth', 2);
hold on;
plot(simulacion_step.out.phi_ref, 'linewidth', 2);
title('Ángulo \phi seguimiento de referencia simulación');
legend('\phi', '\phi ref');
grid on;