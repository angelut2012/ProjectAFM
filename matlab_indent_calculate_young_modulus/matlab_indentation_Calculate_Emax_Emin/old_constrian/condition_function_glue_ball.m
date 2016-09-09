function [c,ceq]=condition_function_glue_ball(x)
F=x(1);
R=x(2);
ds=x(3);
%%
laod_constants

%

dtip=F/k;
A=pi*R*ds;
P_contact=F/A;


% ...<=0
c=[
    F/k-dtip_max
    ds-Htip
    dtip+ds-Zmax*0.9
    P_contact-Ysample
    P_contact-Ytip
    dtip-dtip_saturation
    N*Znoise-dtip
    N*Znoise-ds
    ds-R% for glue ball
    
    % all >0
    -F
    -R
    -ds
    ];
ceq=[
R-50*amp    
% R-10000
];

end