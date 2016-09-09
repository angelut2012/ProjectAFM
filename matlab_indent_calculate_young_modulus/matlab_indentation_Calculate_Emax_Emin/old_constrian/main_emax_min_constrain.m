 clear
 warning on
 format short
% F=x(1);
% R=x(2);
% ds=x(3);
%dcut=x(4);
% amp=1e-9
laod_constants
% RS=500:50:2000
RS=50*amp
for n=1:length(RS)
    R0=RS(n);
x0=[500*amp*40
R0*amp
20*amp
Znoise
];
lb=[N*Znoise*k/2
    50*amp
    N*Znoise/2
    Znoise
];

ub=[];


tor=1e-30;
options = optimoptions('fmincon','Algorithm','interior-point' ,...
   'TolCon',tor,'TolX',tor,'TolFun',tor);
constrain_fun='condition_function_glue_ball'
% constrain_fun='condition_function_FIB_cut'
[x,fval,exitflag,output]=fmincon(@fun_E,x0,[],[],[],[],lb,ub,constrain_fun,options);

F=x(1)
R=x(2)
ds=x(3)
dtip=F/k
E=3/4*x(1)/x(2)^0.5/x(3)^1.5;
E_GPa(n)=E/1e9
end
out=[F R ds dtip E_GPa]