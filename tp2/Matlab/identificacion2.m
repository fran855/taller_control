close all
clc
clear all

load('identificacion_con_imu_chico.mat');

angulo = out.d2;
tiempo = out.tout;

angulo_recortado = angulo(905:1203);