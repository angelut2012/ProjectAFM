 close all
 clear

data=importdata('D:\AFMdata\SEM_drift_after_10_hour_then_start_all.txt');
% data=data.data;

t=data(:,2);
y=data(:,9);
x=data(:,10);
p=data(:,11);
z=data(:,12);

y=y-y(1);
x=-x;
x=x-x(1);
p=p-p(1);
z=z-z(1);

t=t./60;

rx=203090%233590-6719
ry=163642
rz=243502%211968-51845
dx=92.509e3
dy=71.816e3
dz=21.814e3

% dx=80000
% dz=20000
x=x./rx.*dx;
y=z./ry.*dy;
z=z./rz.*dz;
p=p./35;



T=data(:,8);
T=convert_temperature_adc2degree(T);
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

figure
plot(t,q,'.-')
grid on
xlabel('time (minute)')
ylabel('nm')


q=q(35000:end);
[rq,cfL]=line_polyfit_adjust(q,3);

figure
% plot(cfL,'fit');
plot(rq);

std(rq)