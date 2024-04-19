clc
close all

load('practica3_ident.mat');

figure();
plot(t, h);
title('Variación de la altura');
ylabel('h(t)')
xlabel('t')
grid on;
hold on;
% La solución general para P(s) = -K / (s+p) es y(t) = A e^(-pt) - K/p
% Tomamos y(t -> infty) = 0.3747 = - K/p
% OBS: tomamos el 1479 (promedio de t -> infinito) -K/p = 0.3736

% OBS: vamos a tomar el A como el cuarto punto
% Además, y(t = 4) = 0.4624 = A - K/p -> A = 0.4624 - 0.3736 = 0.0888

A = 0.0888;
cte = -0.3736;

for p = 0.0025 : 0.0001 : 0.0035
    y = A * exp(-p * t) - cte;
    plot(t, y)
end

% Elegimos 0.003
p = 0.003;
K = -0.3747 * p;

figure();
plot(t, h);
hold on;
y = A * exp(-p * t) - cte;
plot(t, y)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 2. Discretización de la transferencia
% De Ps = -k/(s+p) -> Pd = kd / (z+pd) -> h_{n+1} = pd h_n + kd u_n
h_centrado = h - h(1);
u_centrado = u - u(1);

dim = length(h);
h_n1 = h_centrado(2:dim);
X = [h_centrado(1:dim-1) u_centrado(1:dim-1)];
param = pinv(X) * h_n1;
pd = param(1);
kd = param(2);

% Sean Ad = pd, Bd = kd
% Ad = e^(AT), Bd = int_0^T e^(A* n) B dn = B * (e(A*T)/A - 1/A)
% ps = -A; ks = -B
T = 1;
A = log(pd)/T;
B = A * kd / (exp(A*T)- 1);
p_s = -A;
k_s = -B;

s = tf('s');
Ps = - k_s / (s + p_s);

% La solución general para P(s) = -K / (s+p) es y(t) = A e^(-pt) - K/p
figure()
plot(t,h)
hold on;
step(Ps*(u(2)-u(1))+h(1))