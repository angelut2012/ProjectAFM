clear
close all
clc
% function AFM_show_indent_data()
% data=load('..\bin\IndentData.txt');
% dat=importdata('..\bin\IndentData.txt');

% dat=importdata('D:\AFMdata\IndentData.txt');
dat=importdata('D:\AFMdata\IndentData_vibration_si_floating_table_20160409173932.txt');


data=dat.data;
% data=load('..\bin\pdms_tf\pdms_IndentData_step_size20_start_position6500_depth3000_time408218771_20150528190459.txt');

% data=load('..\bin\sensitivity_calibration_on_StainStell_IndentDataSiO2_TriggerForce_nN700_LoopDelay_uS1_Step_Size_nm_2_20150611124154.txt');
data(data==0)=nan;
% data=data(1:end-1,:);

nm=data(:,1);
prc=data(:,2);


% L=length(nm);
% Lh=round(L/2);
Lh=find(nm==max(nm));% asysmetric
Lh=Lh(1);
nm1=nm(1:Lh);
nm2=nm(Lh:end);

prc1=prc(1:Lh);
prc2=prc(Lh:end);
% ind=(nm~=0);
% nm=data(ind,1);
% prc=data(ind,2);
% prc(prc>5e6)=nan;

figure(20)
clf
% plot(nm,prc,'.-')
plot(nm1,prc1,'r.-')
hold on
plot(nm2,prc2,'b.-')
grid on
legend('indent','withdraw','Location','NorthWest')
xlabel('indent depth (nm)')
ylabel('sensor readout')


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

prc1=prc2;
prc1=prc1/40;
figure(4)
clf
% plot(nm,prc,'.-')
plot(prc1,'r.-')
[x,y]=ginput(2)
x=round(x);
x(2)=min(x(2),length(prc1))
prc1=prc1(x(1):x(2));
X=1:length(prc1);
cfL=createFit_line_poly_N(X,prc1,1,1)
prc_j=feval(cfL,X);
prc1=prc1-prc_j;
figure(100)
plot(prc1,'*-')


DFFT(prc1,1e6/104);
std(prc1)

return
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
%%


cfL=createFit_line_poly_N(nm1(1:end-200),prc1(1:end-200),1,1)
prc_j=feval(cfL,nm1);
prc1=prc1-prc_j;
figure(100)
plot(prc1,'*-')
% ind=3828:3835;
% ind=3838:3850
ind=3504:3516
ind=3593:3600
ADC=prc1(ind);
depth=nm1(ind);
cf_s=createFit_line_poly_N(depth,ADC,1,1)
cf_s.p1
%  1.017147505215382

return
th=(max(F)-mean(F))*0.01+mean(F);
ind=F>th;
depc=dep(ind);
Fc=F(ind);
figure(2)
clf
plot(dep,F,'.-')
grid on
hold on
plot(depc,Fc,'r.-')
%%
depc=max(depc)-depc;
figure(3)
plot(depc,Fc,'r.-')