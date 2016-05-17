%%
function [outR0,outRange]=sub_calc_show_drift_triangle(w,DW,voltage,t,T);
close all
% w=z;
% DW=dz;


ind0=voltage==0;
ind1=voltage==[262144];
w0=w(ind0);
w1=w(ind1);
t0=t(ind0);
t1=t(ind1);
r=w1-w0;
T0=T(ind0);
rn=r./r(1);
%% 
delta_w0=w0(end)-w0(1)
delta_w0_rate=delta_w0/DW

delta_w0_time_min=delta_w0/t(end)



cf=createFit_line_poly_N(T0,w0,1,1)
rate_w0_T=cf.p1


delta_r=r(end)-r(1)
delta_r_rate=delta_r/DW
delta_r_time_min=delta_r/t(end)

delta_T=T(end)-T(1)


cf=createFit_line_poly_N(T0,r,1,1);
rate_r_T=cf.p1
outR0=[delta_w0 , delta_w0_rate*100, delta_w0_time_min ,rate_w0_T]
outRange=[delta_r , delta_r_rate*100, delta_r_time_min , rate_r_T]

plot(T0,w0)
createFit_line_poly_N(T0,w0,1,1)
ylabel('sensor readout (nm)')
xlabel('temperature (degree centigrade)')



cf=createFit_line_poly_N(T0,r,1,1)
rate_r_T=cf.p1
ylabel('sensor readout (nm)')
xlabel('temperature (degree centigrade)')
return




figure
plot(t0,w0,'r-')
xlabel('time (minute)')
ylabel('nm')
hold on
plot(t1,w1-w1(1),'b.-')

figure
plot(t1,r-r(1),'g-')

legend('0 V','150 V','range')
figure
plot(t1,r./r(1),'gd-')

plot(T0,rn)
plot(t1,rn)
xlabel('time (minute)')
createFit_line_poly_N(T0,r,1,1)

