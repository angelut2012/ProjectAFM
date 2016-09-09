% amp=1e-9;
amp=1;


dtip_max=100e3*amp;% cantilever base yield

Zmax=20e3*amp;%7e3 z piezo max

Ysample=10000;%40e6;
dtip_saturation=200*amp;%7.084e3*amp;% max dtip for PRC readout saturation

N=31;
half_angle_tip=30/2;% dgree
%% un modify Si tip

Znoise=0.5*amp;
R=50;
Ytip=7000;%7 1pa=1e9 nF/nm/nm

% k=40;
% Htip=7.63135e3*amp;% tip height
%% hard
% k=450;
% Htip=50e3*amp;% tip height
% http://www.brukerafmprobes.com/Product.aspx?ProductID=3611

%% soft
k=0.01;
R=20e3
Htip=R;%7e3*amp;% tip height

%%  glass ball
% R=10e3;
% R=2e3;
% Ytip=0.28;% 1pa=1e9 nF/nm/nm
