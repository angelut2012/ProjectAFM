clear
clc
load_tip_sample_parameter
NMax=1000
NL=31
for N=NL:NMax
%% working condition, no yield change on both sample and tip
Pcontact=min(Ysample,Ytip);% contact pressure
%% get Emax
Dsample_min=N*Zstd;
% Dtip=Zmax-Dsample;
% CR=sqrt(R*Dsample);
% A=pi*CR*CR;
A=pi*R*Dsample_min;
Fmax=Pcontact*A;% max contact force
if Fmax>DtipYield*K
  Fmax=DtipYield*K
  disp('PRC base yield');
end
Dtip_max=Fmax/K;

if (Dtip_max>Zmax-Dsample_min)
    error('sample is too hard');
end
Zmotion_min=Dtip_max+Dsample_min;
%%Emin
% %% get Emin
% % before sample yield, tip must have N points to readout
% Dtip_min=N*Zstd;
% Fmin=Dtip_min*K;
% 
% Ex=1/((1-Vsample^2)/Esample+(1-Vtip^2)/Etip);
% F_sample_yield=(Pcontact/0.4)^3*R*R/Ex/Ex;
% if (F_sample_yield<Fmin)
%         error('soft sample yield');
% end
% Dsample_max=Zmax-Dtip_min;
% % if sample is not yield, try more tip displacement
% 
% Dtip_before_sample_yield=F_sample_yield/K
% Dsample_max_before_sample_yield=Zmax-Dtip_before_sample_yield

%% get max min Young's modulus
F=[Fmax ]%,, Fmin, F_sample_yield]
Dsample=[Dsample_min ]%Dsample_max Dsample_max_before_sample_yield]
Ex=F./(4/3*R^0.5.*Dsample.^1.5);
Esample_range=(1-Vsample^2)./(1./Ex-(1-Vtip^2)/Etip);
disp('Zmotion_min for hard sample (nm)'),Zmotion_min*1e9

ExGPa=Esample_range./1e9
Emax(N-NL+1)=ExGPa(1);
end
max(Emax)
plot(Emax)
%%

% Dtip=15.73e-9*12
% k=96.45
% Fmax=Dtip*k
% Ysample=9e6% teflon
% % Ysample=40e6% PS
% P=Ysample
% A=Fmax/P
% R=16e-6;
% Dsample=A/pi/R