g=9.8;
l=0.18;
k=0.004;
m=0.065;
T=0.02;

Ad = eye(2) + [0 1; -g/l -k/(m*l^2)] * T;
Ad2 = eye(3);
Ad2(1:2, 1:2) = Ad;
Cd2 = [1 0 0; 0 1 1];
L = place(Ad2', Cd2', [0.5 0.6 0.95])';


