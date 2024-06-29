K = [0.4473; -0.0038; 0.6962; 0.0023];
H = [0.01; 0];

KH = [K H];
avas = eig(KH);
parteRealNegativa = all(real(avas) < 0);
dentroCirculoUnitario = all(abs(avas) < 1);

if parteRealNegativa
    disp('Todos los AVAS tienen parte real negativa');
end
if dentroCirculoUnitario
    disp('Todos los AVAS están dentro del circulo unitario');
end
