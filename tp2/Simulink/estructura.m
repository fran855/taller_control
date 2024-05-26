%%
datos = struct();
datos.angulo = out.d1;
datos.servo = out.d2;
datos.tiempo = out.tout;

%%
datos7 = struct();
datos7.entrada = double(datos.servo(901:1697));
datos7.salida = double(datos.angulo(901:1697));
datos7.tiempo = double(datos.tiempo(901:1697));

%%
ti = 1;
tf = 10;
% plot(kp0_5ki0_01.tout, kp0_5ki0_01.d1)
subplot(4,1,1)
plot(dopcion22.tout, dopcion22.d1, 'linewidth', 1.5)
xlim([ti,tf]);
xlabel('t [s]');
ylabel('\theta [grados]')
title('Oscilaciones - k_p = 0.4, k_i = 0.1, k_d = 0.001 (sin banda)');
grid on;

subplot(4,1,2)
plot(dopcion22.tout, dopcion22.d2, 'linewidth', 1.5)
xlim([ti,tf]);
xlabel('t [s]');
ylabel('\phi [grados]')
title('Posición del brazo');
grid on;


subplot(4,1,4)
plot(dopcion22.tout, dopcion22.d3, 'linewidth', 1.5)
xlim([ti,tf]);
xlabel('t [s]');
ylabel('i [grados]')
title('Error integral');
grid on;

subplot(4,1,3)
plot(dopcion22.tout, dopcion22.d4, 'linewidth', 1.5)
xlim([ti,tf]);
xlabel('t [s]');
ylabel('d_k [grados/s]')
title('Error derivativo');
grid on;

%%
ti = 0;
tf = 10;
% plot(kp0_5ki0_01.tout, kp0_5ki0_01.d1)
subplot(2,1,1)
plot(out.tout, out.d1, 'linewidth', 1.5, 'color', 'r')
xlim([ti,tf]);
xlabel('t [s]');
ylabel('\theta [grados]')
title('Respuesta al impulso simulada con controlador PID');
grid on;

subplot(2,1,2)
plot(out.tout, out.d2, 'linewidth', 1.5)
xlim([ti,tf]);
xlabel('t [s]');
ylabel('u [grados]')
title('Acción de control');
grid on;