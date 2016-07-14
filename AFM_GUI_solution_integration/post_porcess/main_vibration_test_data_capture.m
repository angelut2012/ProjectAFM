clear
close all

Fs=11.934e3
dat=importdata('D:\AFMdata\data_capture_vibration_test.txt');
prc=dat(:,1)./35;
z=dat(:,2);
prc=line_polyfit_adjust(prc,1);
% prc=prc-mean(prc);
DFFT(prc,Fs,1000);
std(prc)

t= 1:length(prc);
t=t./Fs;
plot(t,prc)
ylabel('vibration amplitude (nm)')
xlabel('time (s)')



z=line_polyfit_adjust(z,1);
std(z)


rx=203090%233590-6719
ry=163642
rz=243502%211968-51845
dx=92.509e3
dy=71.816e3
dz=21.814e3

z=z./158023.*dz
std(z)