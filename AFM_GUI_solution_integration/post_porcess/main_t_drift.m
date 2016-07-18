clear
fn='D:\AFMdata\Zscsg_drift_VS_time.txt'
data=importdata(fn);
for k=1:7
    data(:,k)=data(:,k)-data(1,k);
end
t=data(:,1);
txy=data(:,2);
tz=data(:,3);
sx=data(:,4);
sy=data(:,5);
sp=data(:,6);
sz=data(:,7);
sznm=sz/121642.*10.98e3;
figure(10)
plot(t,sznm)

load('cf_T_z.mat','cf_z_VR2T')
tzc=feval(cf_z_VR2T,tz);

figure(2)
plot(t,tzc)
