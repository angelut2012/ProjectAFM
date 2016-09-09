clear
close all
% fn='PS_200nm.txt'%21
% fn='teflon_10nm.txt'
% fn='brucker_PS_needle_dis1um.txt'
% fn='brucker_PS_needle_dis500nm.txt'
% fn='brucker_PS_needle_dis500nm_no_base_correction.txt'

% Esample=load(fn);
% Esample=Esample./1e3;

% clear
% load PRC_needle_PS_withdraw
% load PRC22_6ball_PS_withdraw_new
% load E_brucker_bprobe7_needle_PS_dis1um_R2
% load E_brucker_bprobe5ball_PS_200nm_R2
% load E_brucker_bprobe5ball_PS_200nm_R2_withdraw
% load E_brucker_bprobe5ball_PS_200nm_R2_extend
% load E_brucker_bprobe5ball_PS_40nm_R2_withdraw
% load ch_E_PS
% Esample=d_PS(:,2:2:end);
% NumberOfPoints=9;
% SamplePerPoints=5;
load PRC_needle_PS_new_stiffness_withdraw_R2

% NumberOfPoints=23;
% SamplePerPoints=5;
ind=gofR2>0.98;
Esample(~ind)=nan;
% Esample(Esample>4.5)=nan;


% clear
% load PRC22_6ball_PS.mat
% NumberOfPoints=11;
% SamplePerPoints=11;

Q=reshape(Esample,SamplePerPoints,NumberOfPoints)
% Qg=reshape(gofR2,SamplePerPoints,NumberOfPoints);
% Q(:,[4 5 ])=[];

% Q(:,[4 5 6 7 9])=[];
Q(:,[4])=[];
% Q(:,7)=[];
%
% Q=Q(:,[1 2 3 7 9]);
%  Q=Q(:,[4 5 6 7 9]);
%%
E=Q;
ES=0.55
ES=3

%% 
E=Q;
for k=1:size(E,2)
[b,inds]=find_outlier_index(E(:,k));
inds
E(inds,k)=nan;
te=E(:,k);
te=te(~isnan(te));
point_mean(k)=mean(te);
point_std(k)=std(te);
end

% std_between_points=std(point_mean)
clc
overall_mean=mean(point_mean)
rate=mean(point_std./point_mean)*100
mean_of_std_ateachpoint=mean(point_std)
std_of_mean=std(point_mean)

return
figure
errorbar(point_mean,point_std)
%  E=E(~isnan(E));
Q=E;
return

%%
limy=max(ES,max(Q(:)) );
%% 
figure
plot(E,'*-')

hold on

plot([1 SamplePerPoints],[ES ES],'k','LineWidth',3)
ylim([0 limy])
xlabel('indentation times')
ylabel('Young''s modulus (GPa)')
xlim([1 SamplePerPoints])
% ylim([min(E(:)) max(E(:))])
figure
boxplot(E)
ylim([0 limy])
grid on
xlabel('point #')
ylabel('Young''s modulus (GPa)')
hold on
plot([0 NumberOfPoints+1],[ES ES],'k','LineWidth',3)


figure
bar(E(1,:))
% ylim([0 0.9])
grid on
xlabel('point #')
ylabel('Young''s modulus (GPa)')
hold on
plot([0 NumberOfPoints+1],[ES ES],'k','LineWidth',3)

ylim([0 limy])

figure
bar(E(end,:))
% ylim([0 0.9])
grid on
xlabel('point #')
ylabel('Young''s modulus (GPa)')
hold on
plot([0 NumberOfPoints+1],[ES ES],'k','LineWidth',3)

ylim([0 limy])
%% 
E=Q(~isnan(Q));
% E=Q;
% for k=1:NumberOfPoints
% [b,inds]=find_outlier_index(E(:,k));
% E(inds,k)=nan;
% end
%  E=E(~isnan(E));

% 

[b,inds]=find_outlier_index(E)
E(b)=[];
E=E(:)%
Em=mean(E)
s=std(E)
s/Em*100
% m=median(E)
% sm=std(E)
% sm/m*100