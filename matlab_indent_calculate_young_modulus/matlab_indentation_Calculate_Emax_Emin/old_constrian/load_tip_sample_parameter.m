
% R=5e-6%
% R= 9.6e-6%
% R=39.63/2*1e-6%50e-9 % tipradius
% R=16e-6
% R=40e-6

R=40e-9

K=40%96.45% cantilever stiffness
DtipYield=16.6e-6% PRC base yield

Zmax=7000e-9% 6 um z piezo motion range
Zstd=0.846e-9%1.2e-9%0.2e-9%0
N=31% number of points
    
%sample property
% % eg. teflon
% Ysample=9e6% sample yield strength
% Vsample=0.46
% Esample=0.5516e9

% %eg. PS
Ysample=40e6% sample yield strength
Vsample=0.34
Esample=3e9


% % eg. Aluminium alloy 2014-T6	
% Ysample=483e6% sample yield strength
% Vsample=0.33 % poisson’s ratio
% Esample=73.1e9% Young’s modulus

% copper	130	117	210
% Ysample=117e6% sample yield strength
% Vsample=0.33 % poisson’s ratio
% Esample=130e9% Young’s modulus


% hard sample
% Ysample=1e20% sample yield strength
% Vsample=0.1
% Esample=200e9

% tip property 
% si
Etip=150e9% tip young's modulus
Ytip=7e9% tip yield strength
Vtip=0.27%0.26~0.28

% PS
% Etip=3e9% tip young's modulus
% Ytip=40e6% tip yield strength
% Vtip=0.34%0.26~0.28
%% glass ball tip
% Etip=60e9% tip young's modulus
% Ytip=0.28e9% tip yield strength
% Vtip=0.2

clc