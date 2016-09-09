clear
close all
load('ch_E_teflon.mat')
load('ch_E_PS.mat')
E=d_PS;
E=d_teflon./1e3;
E=E(:,2:2:end);
N=size(E,2);
plot(1:N,E','*-')

for k=1:N
str{k}=['point ' num2str(k)];
end
legend(str)
grid on
xlabel('mesurement')
ylabel('Young''s modulus (GPa)')

E1=E(1,:);
E2m=mean(E1)
er=max(abs(E1-mean(E1)))
figure

bar(E1,'')
xlabel('point')
ylabel('Young''s modulus (GPa) measured at first time')
Em=mean(E)
Se=std(E)

figure
errorbar(Em,Se)
hold on 
bar(Em)


xlabel('point')
ylabel('mean Young''s modulus (GPa) of multi test on each point')

Ema=mean(E(:))
Sea=std(E(:))
return

fn='teflon_MPa.txt'
d=load(fn);
ind=d(:,1);
E=d(:,2);
g=[14 24 35 46 58 72 81 99]
for k=1:length(g)-1
    
    
    
    
end
% return
figure(2)
errorbar(1:n,Esample,Esample-EL,-Esample+EH)
hold on
bar(1:n,Esample)
% end
% figure(1)
% clf
% plot(Rtip,E,'*-')
grid on
xlabel('mesurement')
ylabel('Young''s modulus (GPa)')
xlim([0.5 n+0.5])
Eq=[Esample-EL;-Esample+EH]
mEq=max(Eq);
[noise_rms_nN,ind]=sort(noise_rms_nN)
mEq=mEq(ind);
noise_rms_nm=noise_rms_nN/para.probe_stiffness_nN_per_NM;
figure(3)
hold on
plot(noise_rms_nm,mEq,'*-')
xlabel('RMS of vibration noise (nm)')
ylabel('error in Young''s modulus (GPa)')
save([filename(1:end-5) '.mat'],'Esample','EH','EL','mEq')