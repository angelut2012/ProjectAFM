 close all
 clear

% data=importdata('D:\AFMdata\SEM_drift_after_10_hour_then_start_all.txt');
% data=importdata('D:\AFMdata\SEM_SCSG_drift_75V.txt');
fn='drift_tri_SEM_newZ_5hour.txt'
fn='drift_air_bag_1.txt'
data=importdata(['D:\AFMdata\' fn]);
% data=data.data;
% 
% data=data(13050:203410,:);
y=1;
t=data(:,1);
% y=data(:,5);
x=data(:,2);
p=data(:,3);
z=data(:,4);

t=t-t(1);
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



T=data(:,4);
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
q=p;
% q=q-min(q);
figure
plot(t,q,'.-')
grid on
xlabel('time (minute)')
ylabel('nm')
return
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