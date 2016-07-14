 close all
 clear

% data=importdata('D:\AFMdata\SEM_drift_after_10_hour_then_start_all.txt');
% data=importdata('D:\AFMdata\SEM_SCSG_drift_75V.txt');
% fn='drift_tri_SEM_newZ_5hour.txt'
% fn='drift_tri_SEMAir_newZ_1hour.txt'
% fn='drift_tri_Air_newZ_1hour.txt'

fn='piezo_hysteresis_triangle_test.txt'
data=importdata(['D:\AFMdata\' fn]);
% data=data.data;
% 
% data=data(16897:end,:);%20 minutes later

data=data(11265+2048*2:end,:);%20 minutes later
% t=data(:,1);
% data=data(:,2:8);
% 
% data=data(4993:end,:);%20 minutes later

V=data(:,1);
y=data(:,4);
x=data(:,5);
% p=data(:,6);
z=data(:,7);





% t=t-t(1);
y=y-y(1);
% x=-x;
x=x-x(1);
% p=p-p(1);
z=z-z(1);

% z=-z;

% t=t./60;

rx=203090%233590-6719
ry=163642
rz=243502%211968-51845
dx=92.509e3
dy=71.816e3
dz=21.814e3

% % dx=80000
% % dz=20000
% x=x./rx.*dx;
% y=z./ry.*dy;
% z=z./rz.*dz;
% % p=p./35;
%% 
q=-y;
q=line_polyfit_adjust(q,1);

ind=1025
v=V(1:ind);
q=q(1:ind);
v=normalize_01(v);
q=normalize_01(q);

cf=createFit_line_poly_N(v,q,7,1)

xlabel('driving voltage')
ylabel('displacement')
ylim([0 1])


Q=feval(cf,v);
d=q-Q;
% d=line_polyfit_adjust(d,1);

ind=1025
ve=v(1:ind);
de=d(1:ind);

figure
plot(ve,de)

xlabel('driving voltage')
ylabel('displacement error')

er=max(abs(de))
ylim([-er er])
return






plot(t0,z0,'r-')
hold on
plot(t1,z1-z1(1),'b.-')
r=z1-z0;

plot(t1,r-r(1),'gd-')

legend('0 V','150 V','range')

plot(t1,r./r(1),'gd-')
T0=T(ind0);
plot(T0,r./r(1))


plot(t,T)





% T=T-T(1);
% T=T.*1000;
plot(t,x,t,y,t,z,t,p,t,T)
legend('SCSG X','SCSG Y','SCSG Z','PRC Readout','temperature')
grid on
xlabel('time (minute)')
ylabel('nm')

figure
plot(t,T)
grid on
xlabel('time (minute)')
ylabel('temperature (degree centigrade)')
title('')


close all
q=z;
q=q-min(q);
figure
plot(t,q,'.-')
grid on
xlabel('time (minute)')

xlabel('temperature (degree centigrade)')
ylabel('nm')
createFit_line_poly_N(T,q,2,1)
return

q=q(1:end);
[rq,cfL]=line_polyfit_adjust(q,5);

figure
% plot(cfL,'fit');
plot(rq);

std(rq(10000:20000))
q(end)/t(end)