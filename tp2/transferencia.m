clear all;
close all;
clc;

syms theta1 theta2 theta1_dot theta2_dot tau1 tau2

% Ecuaciones de movimiento del péndulo de Furuta
eq1 = ...; % Escribe aquí la ecuación de movimiento para theta1_dot
eq2 = ...; % Escribe aquí la ecuación de movimiento para theta2_dot

% Punto de equilibrio (donde las derivadas son cero)
theta1_eq = ...; % Escribe aquí el valor de equilibrio para theta1
theta2_eq = ...; % Escribe aquí el valor de equilibrio para theta2
theta1_dot_eq = ...; % Escribe aquí el valor de equilibrio para theta1_dot
theta2_dot_eq = ...; % Escribe aquí el valor de equilibrio para theta2_dot
tau1_eq = ...; % Escribe aquí el valor de equilibrio para tau1
tau2_eq = ...; % Escribe aquí el valor de equilibrio para tau2

% Definir el vector de variables y ecuaciones
X = [theta1; theta2; theta1_dot; theta2_dot];
U = [tau1; tau2];
F = [eq1; eq2];

% Evaluar el Jacobiano en el punto de equilibrio
A = subs(jacobian(F, X), [X; U], [theta1_eq; theta2_eq; theta1_dot_eq; theta2_dot_eq; tau1_eq; tau2_eq]);
B = subs(jacobian(F, U), [X; U], [theta1_eq; theta2_eq; theta1_dot_eq; theta2_dot_eq; tau1_eq; tau2_eq]);