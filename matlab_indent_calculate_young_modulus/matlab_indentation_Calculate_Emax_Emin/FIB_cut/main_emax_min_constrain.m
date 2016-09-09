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


x0=[50*k
20
50
1
];
lb=[N*Znoise*k
    N*Znoise
    1
    1
];

ub=[];


tor=1e-19;
options = optimoptions('fmincon','Algorithm','interior-point' ,...
   'TolCon',tor,'TolX',tor,'TolFun',tor);
% constrain_fun='condition_function_glue_ball'
constrain_fun='condition_function_FIB_cut'

[x,fval,exitflag,output]=fmincon(@fun_Emax,x0,[],[],[],[],lb,ub,constrain_fun,options);

F=x(1);
ds=x(2);
R=x(3)
dcut=x(4);
dtip=F/k;
E=3/4*F/R^0.5/ds^1.5;
E_GPa=E;

outmax=[F R ds dtip  E_GPa dcut]
%%

[x,fval,exitflag,output]=fmincon(@fun_Emin,x0,[],[],[],[],lb,ub,constrain_fun,options);

F=x(1);
% R=x(2)
ds=x(2);
dtip=F/k;
E=3/4*F/R^0.5/ds^1.5;
E_GPa=E;

outmin=[F R ds dtip  E_GPa dcut]
outall=[outmax;outmin]