function [ExGPa,P]=calc_YoungsModulus_judge(Dtip,Dsample,Ytip,Ysample,Vtip,Vsample,R,K)
F=K*Dtip;% force
CR=sqrt(R*Dsample);
A=pi*CR*CR;
P=F/A% contact pressure for judge
if (P>Ysample)
    disp('sample yield');
end
if (P>Ytip)
    disp('tip yield');
end
Ex=F/(4/3*R^0.5*Dsample^1.5);
ExGPa=Ex/1e9;
% Ecalc=(1-Vsample^2)/(1/Ex-(1-Vtip^2)/Etip)
end
