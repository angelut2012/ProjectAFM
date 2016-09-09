function [c,ceq]=condition_function_FIB_cut(x)
F=x(1);

ds=x(2);
R=x(3);

dcut=x(4);
%%
laod_constants

%

dtip=F/k;
A=pi*R*ds;
P_contact=F/A;
tg=tand(half_angle_tip);
% syms R tg dc y
% solve('(R*R-(R-y)^2)/tg-dc'==y)

arc_depth=R - tg/2 + (4*R^2 - 4*R*tg + tg^2 - 4*dcut*tg)^(1/2)/2;


% ...<=0
c=[
    F/k-dtip_max
    ds-Htip
    dtip+ds-Zmax
    P_contact-Ysample
    P_contact-Ytip
    dtip-dtip_saturation
    N*Znoise-dtip
    N*Znoise-ds
%     ds-R% for glue ball
% FIB cut
dtip+ds+dcut-Htip
ds-arc_depth
    
    % all >0
    -F
    -R
    -ds
    ];
ceq=[];

end