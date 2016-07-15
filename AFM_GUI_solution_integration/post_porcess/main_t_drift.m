clear
fn='D:\AFMdata\drift_Test2.txt'
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
figure(1)
plot(t,sz)
figure(2)
plot(t,tz)
