% amp=1e-9;
amp=1;

k=40;
dtip_max=16.6e3*amp;% cantilever base yield
Htip=7.63135e3*amp;% tip height
Zmax=7e3*amp;% z piezo max

Ysample=10000;%40e6;
dtip_saturation=7.084e3*amp;% max dtip for PRC readout saturation
Znoise=0.8*amp;
N=31;
half_angle_tip=30/2;% dgree
%% un modify Si tip
% Ytip=7;% 1pa=1e9 nF/nm/nm
% R=50;
%%  glass ball
R=10e3;
R=2e3;
Ytip=0.28;% 1pa=1e9 nF/nm/nm
