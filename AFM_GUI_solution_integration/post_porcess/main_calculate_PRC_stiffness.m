clear
close all
clc
% function AFM_show_indent_data()
% data=load('..\bin\IndentData.txt');
% dat=importdata('..\bin\IndentData.txt');

dat=importdata('C:\AFMdata\IndentData.txt');
data=dat.data;
% data=load('..\bin\pdms_tf\pdms_IndentData_step_size20_start_position6500_depth3000_time408218771_20150528190459.txt');

% data=load('..\bin\sensitivity_calibration_on_StainStell_IndentDataSiO2_TriggerForce_nN700_LoopDelay_uS1_Step_Size_nm_2_20150611124154.txt');
data(data==0)=nan;
% data=data(1:end-1,:);

nm=-data(:,1);
prc=data(:,2);


% vH=nm;
%               vH0 = 1.435565217391304e+04;
%               vh_range = 1.519552569169960e+05;
% 
%             vH = (vH - vH0) ./ vh_range;
%             MAX_RANGE_Z_NM = (21.04 * 1000.0);
%             vH =vH.* MAX_RANGE_Z_NM;
% nm=vH;

% L=length(nm);
% % Lh=round(L/2);
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
% plot(nm,'r.-')
% hold on
plot(prc,'b.-')
grid on

[a,b]=ginput(2)
a=round(a);
ind=a(1):a(2);

R=120398 
nm=nm./R.*9365


s_nm=nm(ind);
s_prc=prc(ind);

cfL=createFit_line_poly_N(s_nm,s_prc,1,1)

