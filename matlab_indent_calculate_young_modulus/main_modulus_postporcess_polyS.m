clear
clc
close all
load Poly_Modulus
ch=1:length(Esample);
em=([4 2 13 ])
for k=ch
%     k=4
s=gofR2{k}>0.8
s=s(2:end)
E=Esample{k}(s)'
ind=find_outlier_index(E)
E(ind)=[]
% E=E(isreal(E))
N(k)=length(E)
Eout(k)=mean(E)
Sigma(k)=std(E)
R=gofR2{k}(s)'

end
figure(1)
bar(Eout)
hold on
errorbar(Eout,Sigma)
Eout(em)=[]
Sigma(em)=[]
figure(2)
bar(Eout)
hold on
errorbar(Eout,Sigma)

ylabel('Young''s modulus (GPa)')
xlabel('point #')
mean(Eout)