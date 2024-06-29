%%
close all
clc
clear all

s = tf('s');
T = 0.01;

a = 250;
b = 25;
c = a;

d = 0.8529;
e = 1.046;
f = 57.65;

Gservo = tf(a, [1 b c]);
Gpendulo = tf(d, [1 e f]);
Gtotal = Gpendulo * Gservo;

A = [0 1 0 0; 
     -f -e -c*d -b*d; 
     0 0 0 1; 
     0 0 -c -b];
 
B = [0; a*d; 0; a];

C = [1 0 0 0;
    0 0 1 0];

D = 0;

%H = C*inv(s*eye(4)-A)*B + D;

Ad = eye(4) + A * T;
Bd = B*T;
Cd = C;
Dd = D;

%% K y F
K = [-0.5, 0.001, 0.7, -0.005];
F = pinv(Cd * inv(eye(size(Ad)) - (Ad + Bd * K)) * Bd);

%% H
Cdaux = [0,0,1,0];
I = eye(size(Cdaux, 1));
A_integral = [
    Ad, zeros(size(Ad, 1), size(I, 1)); % Ad en la esquina superior izquierda y ceros a su derecha
    -Cdaux, I                              % Cd en la parte inferior izquierda y I a su derecha
];

B_integral = [
    Bd;
    zeros(1, 1)
];


polos = [real(polos_place_K) 0.945];

H = place(A_integral, -B_integral, polos3);






%% K

% Rango de valores para probar los elementos de la matriz K
k_min = 0;
k_max = 0.8;
num_steps = 1000;
k_values = linspace(k_min, k_max, num_steps);

% Inicializar un contador para las matrices K que cumplen la condición
valid_K_count = 0;

% Bucles anidados para recorrer todas las combinaciones posibles de K
for k1 = k_values
    for k2 = k_values
        for k3 = k_values
            for k4 = k_values
                % Define tu matriz K (ajusta según el tamaño y forma necesarios)
                K = [k1, k2, k3, k4];
            
                % Calcula la matriz del sistema cerrado
                A_cl = Ad - Bd * K;
            
                % Calcula los autovalores del sistema cerrado
                eigenvalues = eig(A_cl);
            
                % Verifica si todos los autovalores están dentro del círculo unitario
                if all(abs(eigenvalues) < 1)
                    fprintf('Matriz K válida: [%f, %f, %f, %f]\n', k1, k2, k3, k4);
                    valid_K_count = valid_K_count + 1;
                end
            end
        end
    end
end

fprintf('Número total de matrices K válidas: %d\n', valid_K_count);

%% K2

K = place(Ad, Bd, [0.20, 0.3, 0.5, 0.6]);
K