close all
clear
clc

% data=importdata('D:\AFMdata\SEM_drift_after_10_hour_then_start_all.txt');
% data=importdata('D:\AFMdata\SEM_SCSG_drift_75V.txt');
fn='drift_tri_SEM_newZ_5hour.txt'
% fn='drift_tri_SEMAir_newZ_1hour.txt'
% fn='drift_tri_Air_newZ_1hour.txt'

% fn='test_triangle_wave6.txt'

fn='T_compensation_null_sensitivity_Y_SEM_vacuum_50_150V_70min_mat.txt'

fn='matching_scsg_Y_SEM_air_0V_50V_20min.txt'
fn='matching_scsg_Y_SEM_vacuum_0V_50V_after10hourInVacuum_8min.txt'
fn='noise_of_Yaxis.txt'
fn='data_xyz.txt'
fn='matching_scsg_Y_SEM_vacuum_VCC1d25_piezo0_150.txt'
fn='matching_scsg_Y_SEM_vacuum_after3hour_VCC1d25_piezo0_150.txt'

fn='matching_scsg_Y_SEM_vacuum_VCC1d25_piezo0_150_overnight_after_V75.txt'

fn='matching_scsg_Y_SEM_vacuum_VCC1d25_piezo0_150_overnight_trianglewave.txt'
fn='scsg_comp_X_correct_3hour.txt'
fn='scsg_comp_X_correct.txt'
% fn='scsg_comp_X_correct_square_wave_after10hours_1.txt'

fn='scsg_comp_X_correct_ShortPiezo_1hour.txt'

fn='scsg_comp_X_ShortPiezo_null_only.txt'

fn='new_scsg_comp_X_ShortPiezo_null_only_dT1.txt'

fn='new_scsg_comp_X_ShortPiezo_null_only_V1d25.txt'

fn='new_scsg_comp_X_ShortPiezo_null_only_V2d5_recalibration_process.txt'
fn='new_scsg_comp_X_ShortPiezo_null_only_V2d5_recalibration_SEM_process.txt'
fn='new_scsg_comp_X_ShortPiezo_null_only_V2d5_recalibration_SEM_12hours_nomove_process.txt'
% fn='new_scsg_comp_X_ShortPiezo_null_only_V2d5_recalibration_16hours_nomove_heatplate_processy.txt'
data=importdata(['D:\AFMdata\' fn]);
% data=data.data;
%
% data=data(16897:end,:);%20 minutes later

% data=data(8193:end,:);%20 minutes later
% data=data(16100:end,:);%20 minutes later
% data=data(512:end,:);%20 minutes later

% data=data(1270:end,:);%20 minutes later
% data=data(1:6600,:);%20 minutes later
% data=data(396:end,:);%20 minutes later
% data=data(396:7000,:);%20 minutes later

t=data(:,1);
t=t-t(1);
% ind=find(t>30*60);
% ind=ind(1:28156)
% data=data(ind,:);
t=data(:,1);
V=data(:,2);
y=data(:,5);
x=data(:,6);
p=data(:,7);
z=data(:,8);





t=t-t(1);
t=t./60;
y=-y;
y=y-min(y);




rx=203090%233590-6719

% 230010-112176%119709%67656% [225060-107346]%31250%89898% 163642
rz=243502%211968-51845
dx=92.509e3
dy=71.816e3
dz=21.814e3
%%

dy=  9.177734375000000e+04/3%925*99.21875
% ry= 201054-27310
ry=57308-54466
ry= 5.233075814400000e+05
ry=(248892-57508)

x=x./rx.*dx;
y=y./ry.*dy;
z=z./rz.*dz;
p=p./35;

figure(10)
plot(t,y)
xlabel('time (minute)')
ylabel('nm')
% return

% y0=y(35000:end);
% x=-x;
% x=x-x(1);
% p=p-p(1);
% z=z-z(1);

% z=-z;






T=data(:,4);
T=convert_temperature_adc2degree(T);

figure
cf=createFit_line_poly_N(t,y,2,1)

ylabel('nm')
title (['drift rate = ' num2str(cf.p2) ' nm/min'])
xlabel('time (min)')

grid on
%%
% [outR0,outRange]=sub_calc_show_drift_triangle(p,1,V,t,T)
% [outR0,outRange]=sub_calc_show_drift_triangle(z,dz,V,t,T)
% [outR0,outRange]=sub_calc_show_drift_triangle(x,dx,V,t,T)
% [outR0,outRange]=sub_calc_show_drift_triangle(y,dy,V,t,T)

% r00=85890
% y=y/r00*
figure(20)
plot(t,T)
xlabel('time (minute)')
ylabel('temperature (degree centigrade)')

cf=createFit_line_poly_N(T,y,1,5)
xlabel('temperature (degree centigrade)')
ylabel('nm')
title (['drift rate = ' num2str(cf.p1) ' nm/^oC'])

figure(21)

[hAx,hLine1,hLine2] =plotyy(t,T,t,y)

xlabel('time (minute)')

ylabel(hAx(1),'temperature (degree centigrade)') % left y-axis
ylabel(hAx(2),'sensor readout (nm)') % right y-axis


return
[outR0,outRange]=sub_calc_show_drift_triangle(y,dy,V,t,T)






return
q=-z;
q=line_polyfit_adjust(q,1);

v=normalize_01(V);
q=normalize_01(q);

createFit_line_poly_N(v,q,1,1)
return

q=q(1:end);
[rq,cfL]=line_polyfit_adjust(q,1);





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