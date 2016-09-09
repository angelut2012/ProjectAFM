clear
clc
R=40e-9

% max
Fm=2.85e5*1e-9
d=0.846e-9*31
% si
Etip=150e9% tip young's modulus
Ytip=7e9% tip yield strength
Vtip=0.27%0.26~0.28

%% glass ball tip
% Etip=60e9% tip young's modulus
% Ytip=0.28e9% tip yield strength
% Vtip=0.2

A=pi*R*d;
Fmax=Ytip*A;% max contact force

F=min(Fm,Fmax)


% %min
% d=6000e-9
% F=3.9e-9

E=3/4*F/d^1.5/R^0.5
EG=E/1e9