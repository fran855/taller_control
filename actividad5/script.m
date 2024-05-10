clear all;

% Función de transferencia de la planta
num = [-0.099863, 0];
den = [1, 0, -(10.14)^2];
plant = tf(num, den);

% Ajustar los parámetros del controlador PID
[controller, info] = pidtune(plant, 'PID');

% Mostrar los parámetros del controlador PID ajustados
display(controller);

% Respuesta al escalón de la planta controlada
sys_cl = feedback(controller * plant, 1);
step(sys_cl);
title('Respuesta al escalón de la planta controlada');
grid on;

[a9