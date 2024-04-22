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
g = 9.8;        %En metros sobre segundo cuadrado
H = 0.9;        %En metros
L_s = 0.4;      %En metros
L_i = 0.1;      %En metros
Q_i = 0.0001333; %Metros cubicos por segundo
d2 = 10.65e-3;
A_o = pi * (d2 / 2)^2;

%Puntos de equilibrio
h0 = 0.45;
u0 = Q_i / (A_o * sqrt(2 * g * h0));
h = h0;
u = u0;

A_eval = eval(A);
B_eval = eval(B);
C_eval = eval(C);
D_eval = eval(D);

planta = simplify(C_eval*(s - A_eval)^(-1) * B_eval + D_eval);
planta2 = C_eval*(s - A_eval)^(-1) * B_eval + D_eval;
