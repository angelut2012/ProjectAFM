% main_error_propagation
clear
format short
close all
syms dt ds sdt sds
k=40
R=4.87e3%50%
%%
n=3% 99.7% confidence
th=0.05% normalized error limit


L=1,H=6000
N=500;

sdt=0.683+0.211
szp=0.11

sds=sqrt(sdt^2+szp^2)

Es=k*dt/(4/3*R^0.5*ds^1.5)
Dt=diff(Es,'dt')*sdt
Ds=diff(Es,'ds')*sds

sEs=sqrt(Dt^2+Ds^2)


sEs=simple(sEs)
pretty(sEs)
ezsurf(sEs,[0.1 300 0.1 250])


pE=n*sEs/Es
% pretty(sEs)

ezsurf(pE,[L H L H])

% =matlabFunction(pE)


fvpe=inline(vectorize(pE));
mdt=linspace(L,H,N);
mds=linspace(L,H,N);
[Mdt,Mds]=meshgrid(mdt,mds);
pErr=feval(fvpe,Mds,Mdt);
figure
mesh(pErr)

ind=pErr>th;
Mds(ind)=nan;
Mdt(ind)=nan;

ind=(Mds+Mdt)>H;
Mds(ind)=nan;
Mdt(ind)=nan;

fE=inline(vectorize(Es));
fEv=feval(fE,Mds,Mdt);
figure
mesh(Mds,Mdt,fEv)
xlabel('d_{sample} (nm)')
ylabel('d_{tip} (nm)')
zlabel('measurable E (GPa)')

view([-48 40]) %[-172,24]

%% varable ROI
figure
imshow(~isnan(flipud(fEv)),[])
xlabel('d_{sample} (nm)')
ylabel('d_{tip} (nm)')

%% 
fEv(isnan(fEv))=[];
% sigmaE=subs(pE,{ds,dt},{Mds,Mdt});
% sigmaE=double(sigmaE);
Emax=max(fEv)
Emin=min(fEv)*1e3
