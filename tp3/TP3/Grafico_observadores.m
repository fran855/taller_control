clc;
close all;
clear all;

%Observadores = load('Observadores.mat');
Observadores = load('Sin_controlador.mat');

figure();
subplot(4,1,1);
plot(Observadores.out.tout, Observadores.out.phi, 'linewidth', 1.5);
hold on;
plot(Observadores.out.tout, Observadores.out.phi_est, 'linewidth', 1.5);
legend;
%xlim([1 7]);
xlim([125 167]);
title('Ángulo \phi');
legend('\phi', '\phi estimado');
grid on;

subplot(4,1,2);
plot(Observadores.out.tout, Observadores.out.theta, 'linewidth', 1.5);
hold on;
plot(Observadores.out.tout, Observadores.out.theta_est, 'linewidth', 1.5);
legend;
%xlim([1 7]);
xlim([125 167]);
title('Ángulo \theta');
legend('\theta', '\theta estimado');
grid on;

subplot(4,1,3);
plot(Observadores.out.tout, Observadores.out.dphi_est, 'linewidth', 1.5);
%xlim([1 7]);
xlim([125 167]);
title('Velocidad angular \phi');
legend('Velocidad angular \phi estimado');
grid on;

subplot(4,1,4);
plot(Observadores.out.tout, 2*Observadores.out.dtheta, 'linewidth', 1.5);
hold on;
plot(Observadores.out.tout, Observadores.out.dtheta_est, 'linewidth', 1.5);
legend;
%xlim([1 7]);
xlim([125 167]);
title('Velocidad angular \theta');
legend('Velocidad angular \theta', 'Velocidad angular \theta estimado');
grid on;

figure();
plot(Observadores.out.tout, 2*Observadores.out.dtheta, 'linewidth', 1.5);
hold on;
plot(Observadores.out.tout, Observadores.out.dtheta_est, 'linewidth', 1.5);
legend;
%xlim([1 7]);
xlim([125 167]);
title('Velocidad angular \theta');
legend('Velocidad angular \theta', 'Velocidad angular \theta estimado');
grid on;