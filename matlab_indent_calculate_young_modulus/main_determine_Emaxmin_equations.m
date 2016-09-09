% main_error_propagation
clear
syms F Ex R depth dtip Z V S k
syms v_1 v_2 E_1 E_2 Ex




F=4/3*R^0.5*Ex*depth^(1.5)
F=simple(F)
Ex=solve(F=='F','Ex')
Ex=subs(Ex,'F','k*dtip')
Ex=subs(Ex,'depth','Z-dtip')
Ex=subs(Ex,'dtip','V/S')

Ex=simple(Ex)
% return
EX=1/((1-v_1*v_1)/E_1+(1-v_2*v_2)/E_2)
E2=solve(EX=='EX','E_2')
E2=subs(E2,'EX',Ex)
E2=simple(E2)
% pretty(E2)
% E2=Ex;
% E_2=\frac{{E^*}\, E_{1} - {E^*}\, E_{1}\, {v_{2}}^2}{{E^*}\, {v_{1}}^2 - {E^*} + E_{1}}

syms Delta_S Delta_k Delta_V Delta_R Delta_Z
Dk=diff(E2,'k')*Delta_k
DV=diff(E2,'V')*Delta_V
DS=diff(E2,'S')*Delta_S
DR=diff(E2,'R')*Delta_R
DZ=diff(E2,'Z')*Delta_Z

Sigma=sqrt(Dk^2+DV^2+DS^2+DR^2+DZ^2)
% Sigma=abs(Dk)+abs(DV)+abs(DS)+abs(DR)+abs(DZ)
Sigma=simple(Sigma)

% Sigma=subexpr(Sigma,'W')
% w=latex(Sigma)
pretty(Sigma)
% load data_Zpiezo_PRC_graphite.mat

rms_prc=36.64702531
% n=1.959
%  for q=2:length(PRC)
% q=535
% z=Z_piezo(q);
% prc_read= PRC(q);
Rtip=50
%%

Est=subs(Sigma,{S,k,R,...
Delta_S,Delta_k,Delta_R,E_1,v_1,v_2
},...
{36.2446,40,Rtip,...
0,0,0,135,0.15,0.17
})
return
%%



er=subs(Sigma,{S,k,V,R,Z...
Delta_S,Delta_k ,Delta_V ,Delta_R ,Delta_Z,E_1,v_1,v_2
},...
{36.2446,40,prc_read,         62,z,...
2.826499,20,rms_prc*n,  29.759,z*0.05,135,0.15,0.17
});
Err(q)=double(er);
Err(q)
%  end
% Err(1)=nan;
% figure(3)
% clf
% plot(F_sim,Err,'.-')
% grid on
% ylim([0 40])
% xlabel('indentation force (nN)')
% ylabel('error in final Young''s modulus (GPa)')
% hold on 
% ind=find(Err==min(Err));
% plot(F_sim(ind),Err(ind),'r*','MarkerSize',20)
Err=[];

ww={Dk,DV,DS,DR,DZ}
for a=1:length(ww)
er=subs(ww{a},{S,k,V,R,Z...
Delta_S,Delta_k ,Delta_V ,Delta_R ,Delta_Z,E_1,v_1,v_2
},...
{36.2446,40,prc_read,         62,z,...
2.826499,5,rms_prc*n,  29.759,z*0.05,135,0.15,0.17
});
Err(a)=double(er);
end
aErr=abs(Err)
sqrt(sum(Err.^2))
[aErr,ind]=sort(aErr)
bar(aErr)
ylabel('propagation error contributing on final Young''s modulus (GPa)')

ylabel('propagation error (GPa)')