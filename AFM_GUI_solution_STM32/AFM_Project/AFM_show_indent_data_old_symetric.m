% clear
% close all
% clc
function AFM_show_indent_data()
data=load('..\bin\IndentData.txt');

% data=load('..\bin\pdms_tf\pdms_IndentData_step_size20_start_position6500_depth3000_time408218771_20150528190459.txt');

% data=load('..\bin\IndentData_step_size2_start_position6900_depth300_time103935944_20150518180742.txt');
data(data==0)=nan;
% data=data(1:end-1,:);

nm=data(:,1);
prc=data(:,2);


L=length(nm);
Lh=round(L/2);
nm1=nm(1:Lh);
nm2=nm(1+Lh:end);

prc1=prc(1:Lh);
prc2=prc(1+Lh:end);
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
legend('indent','withdraw')
figure(5)
clf
plot(nm,'r.-')
hold on
plot(prc,'b.-')
grid on
end