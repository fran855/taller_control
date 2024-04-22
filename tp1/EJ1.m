%% Transferencia linealizada

clear all;
close all;
clc;

syms H L L_s L_i A_o Q_i h u g u0 h0 s;

f = (H^2 * (Q_i - A_o * sqrt(2*g*h)*u))/(h^2*(L_s-L_i)^2 + 2*h*L_i*(L_s-L_i)*H+L_i^2*H^2) ;
y = h; %Salida del sistema

%Genero las matrices de estados
A = jacobian(f, h);
B = jacobian(f, u);
C = jacobian(y, h);
D = jacobian(y, u);

%Definicion de constantes
g = 9.8;               % m/s^2
H = 0.9;               % m
L_s = 0.4;             % m
L_i = 0.1;             % m
Q_i = 0.0001333;       % m^3/s
d2 = 10.65e-3;         % m
A_o = pi * (d2 / 2)^2; % m^2

%Puntos de equilibrio
h0 = 0.45;
u0 = Q_i / (A_o * sqrt(2 * g * h0));
h = h0;
u = u0;

A_eval = eval(A);
B_eval = eval(B);
C_eval = eval(C);
D_eval = eval(D);

planta = zpk(ss(A_eval, B_eval, C_eval, D_eval));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 2. Discretización de la transferencia
clear all;
load('practica3_ident.mat');

T = 1;
p_transf = 0.00237;
pd_transf = exp(-T*p_transf);
% ps = -log(pd)/T -> pd = e^(-T ps)
% De Ps = k_s/(s+p) -> Pd = kd / (z+pd) -> h_{n+1} = pd h_n + kd u_n
h_centrado = h - h(1);
u_centrado = u - u(1);

dim = length(h);
h_n1 = h_centrado(2:dim);
X = [u_centrado(1:dim-1)];
param = pinv(X) * (h_n1 - pd_transf * h_centrado(1:dim-1));

kd = param;

% Sean Ad = pd, Bd = kd
% Ad = e^(AT), Bd = int_0^T e^(A* n) B dn = B * (e(A*T)/A - 1/A)
% ps = -A; ks = -B
% k_s = - A k_d/(exp(AT)-1) = - log(p_d)/T k_d/(exp(log(pd)/T T - 1) =
% -log(p_d)/T k_d/(p_d - 1)

A = log(pd_transf)/T;
B = A * kd / (exp(A*T)- 1);
p_s = -A;
k_s = -B;

A_o = k_s/47.5176;
s = tf('s');
Ps2 = - k_s / (s + p_s);

rpta = step(t, Ps2*(u(2)-u(1))+h(1), 'r');
figure;
plot(t, h, 'b', 'LineWidth', 1.5);
hold on;
plot(t, rpta, 'r', 'LineWidth', 1.5);
title('Respuesta al escalón de la planta identificada');
grid on;
legend('Datos', 'Respuesta al escalón');
xlabel('Tiempo (s)');
ylabel('Amplitud');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Distintos puntos de trabajo
clear all;
close all;
clc;

syms H L L_s L_i A_o Q_i h u g u0 h0 s;

f = (H^2 * (Q_i - A_o * sqrt(2*g*h)*u))/(h^2*(L_s-L_i)^2 + 2*h*L_i*(L_s-L_i)*H+L_i^2*H^2) ;
y = h; %Salida del sistema

%Genero las matrices de estados
A = jacobian(f, h);
B = jacobian(f, u);
C = jacobian(y, h);
D = jacobian(y, u);

%Definicion de constantes
g = 9.8;               % m/s^2
H = 0.9;               % m
L_s = 0.4;             % m
L_i = 0.1;             % m
Q_i = 0.0001333;       % m^3/s
d2 = 10.65e-3;         % m
A_o = pi * (d2 / 2)^2; % m^2

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

figure;
colores = {'b', 'r', 'g', 'm', 'c', 'k', 'y', 'b--', 'r--', 'g--', 'm--', 'c--', 'k--', 'y--'};

hold on;
for i = 1:length(P_list)
    P = P_list{i};
    bode(P, colores{i});
end

hold off;   
legend('h_0 = 0.10', 'h_0 = 0.20', 'h_0 = 0.30', 'h_0 = 0.40', 'h_0 = 0.50', 'h_0 = 0.60', 'h_0 = 0.70', 'h_0 = 0.80');  % Ajusta las leyendas según tus sistemas
title('Diagrama de Bode');
grid on;