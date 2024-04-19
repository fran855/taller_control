syms H L L_s L_i A_o Q_i h u g u0 h0 s;

f = (H^2 * (Q_i - A_o * sqrt(2*g*h)*u))/(h^2*(L_s-L_i)^2 + 2*h*L_i*(L_s-L_i)*H+L_i^2*H^2) ;
y = h; %Salida del sistema

%Genero las matrices de estados
A = jacobian(f, h);
B = jacobian(f, u);
C = jacobian(y, h);
D = jacobian(y, u);

%Definicion de constantes
g = 9.8;        %En metros sobre segundo cuadrado




H = 0.9;        %En metros
L_s = 0.4;      %En metros
L_i = 0.1;      %En metros
Q_i = 0.0001333; %Metros cubicos por segundo
d2 = 10.65e-3;
A_o = pi * (d2 / 2)^2;

%Puntos de equilibrio
h0_valores = [0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70, 0.80];
P_list = cell(1, length(h0_valores));

for i = 1 : length(h0_valores)
    h0 = h0_valores(i);
    u0 = Q_i / (A_o * sqrt(2 * g * h0));
    h = h0;
    u = u0;

    A_eval = eval(A);
    B_eval = eval(B);
    C_eval = eval(C);
    D_eval = eval(D);

    P = zpk(ss(A_eval,B_eval,C_eval,D_eval));
    P_list{i} = P;
end

% Crear una figura para el diagrama de Bode
figure;

% Crear un arreglo de colores para distinguir cada sistema en el diagrama
colores = {'b', 'r', 'g', 'm', 'c', 'k', 'y', 'b--', 'r--', 'g--', 'm--', 'c--', 'k--', 'y--'};

hold on;

% Iterar sobre cada sistema en P_list y trazar su diagrama de Bode
for i = 1:length(P_list)
    % Obtener el sistema actual
    P = P_list{i};
    
    % Trazar el diagrama de Bode para el sistema actual con un color específico
    bode(P, colores{i});
end

hold off;   

% Agregar leyenda y título al diagrama
legend('P1', 'P2', 'P3', 'P4', 'P5', 'P6', 'P7', 'P8');  % Ajusta las leyendas según tus sistemas
title('Diagrama de Bode para sistemas en P\_list');