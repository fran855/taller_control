% Función de transferencia de la planta
num = [-0.099863, 0];
den = [1, 0, -(10.14)^2];
P = tf(num, den);

% Abrir la aplicación "Control System Designer"
controlSystemDesigner(P);


