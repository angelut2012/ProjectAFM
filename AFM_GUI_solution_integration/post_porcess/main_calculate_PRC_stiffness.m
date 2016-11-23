clear
close all
p=('C:\AFMdata\')

p='C:\AFMdata\Measure_PRC_stiffness_force_load_inSEM_20160907\'
fn='IndentData_PRC_stiffness_end*.txt'
fn='IndentData_*.txt'

[fn,p]=uigetfile([p fn])
fns=dir([p fn])
for k=1:length(fns)
    fn=fns(k).name
    pfn=[p fn];
    
    dat=importdata(pfn);
    data=dat.data;
    data(data==0)=nan;
    nm=data(:,1);
    prc=data(:,2);
    
    Lh=find(nm==max(nm));% asysmetric
    nm1=nm(1:Lh);
    nm2=nm(Lh:end);
    
    prc1=prc(1:Lh);
    prc2=prc(Lh:end);
    
    figure(4)
    clf
    plot(nm,prc,'.-')
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
     Overall_sensitivity=abs(cfL.p1)
%      Overall_sensitivity=10
    %% use force load method to calculate the stiffness of PRC
    kr=34.5380*cosd(13)
    kr=20
    PRC_sensitivity= 1.3073e+02
    
   
    %%
    k_prc=kr*(PRC_sensitivity/Overall_sensitivity-1)
    
    L0=134
    Lt=134-5%124+5
    kout(k)=k_prc/(L0/Lt)^3
    kout_orginal(k)=kout(k)*cosd(13)
end

std(kout)
mean(kout)