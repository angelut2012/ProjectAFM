% clear
% close all
% clc
% function AFM_show_indent_data()
 close all
 clear
% data=load('..\bin\IndentData.txt');
% dat=importdata('..\bin\IndentData.txt');
dat=importdata('C:\AFMdata\IndentData.txt');
if length(dat)==1
    data=dat.data;
else
    data=dat;% compatible with old version
end
% data=load('..\bin\pdms_tf\pdms_IndentData_step_size20_start_position6500_depth3000_time408218771_20150528190459.txt');

% data=load('..\bin\IndentData_step_size2_start_position6900_depth300_time103935944_20150518180742.txt');
data(data==0)=nan;
% data=data(1:end-1,:);

nm=data(:,1);
prc=data(:,2);


% L=length(nm);
% Lh=round(L/2);
Lh=find(nm==max(nm));% asysmetric
nm1=nm(1:Lh);
nm2=nm(Lh:end);

prc1=prc(1:Lh);
prc2=prc(Lh:end);
% ind=(nm~=0);
% nm=data(ind,1);
% prc=data(ind,2);
% prc(prc>5e6)=nan;

figure(4)
clf
% plot(nm,prc,'.-')
plot(nm1,prc1,'r.-')
hold on
plot(nm2,prc2,'b.-')
grid on
legend('indent','withdraw','Location','NorthWest')
xlabel('indent depth (nm)')
ylabel('sensor readout')
figure(5)
clf
plot(nm,'r.-')
hold on
plot(prc,'b.-')
grid on
legend('z piezo (nm)','sensor readout')
% end