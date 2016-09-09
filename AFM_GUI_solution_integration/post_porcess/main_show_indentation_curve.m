clear
close all
p=('C:\AFMdata\poly\')

p=('C:\AFMdata\ptfe\')
p=('C:\AFMdata\')
fn='IndentData_PRC_stiffness_end*.txt'
fn='IndentData*.txt'


[fn,p]=uigetfile([p fn])
fns=dir([p fn])
for k=1:length(fns)
    fn=fns(k).name
    pfn=[p fn];
    
    dat=importdata(pfn);
    data=dat.data;
    % data=load('..\bin\pdms_tf\pdms_IndentData_step_size20_start_position6500_depth3000_time408218771_20150528190459.txt');
    
    % data=load('..\bin\sensitivity_calibration_on_StainStell_IndentDataSiO2_TriggerForce_nN700_LoopDelay_uS1_Step_Size_nm_2_20150611124154.txt');
    data(data==0)=nan;
    % data=data(1:end-1,:);
    
    nm=data(:,1);
    prc=data(:,2);
    
   nof=5
        nm=medfilt2(nm,[nof,1],'symmetric');
        prc=medfilt2(prc,[nof,1],'symmetric');
    
    nm=nm-nm(1);
    prc=prc-prc(1);
    
    R=117440 ;%120398;
    nm=nm./R.*9365;
    
    stiff=40;
    sen=130;
    prc=prc./sen.*stiff;
    
    
    Lh=find(nm==max(nm));% asysmetric
    nm1=nm(1:Lh);
    nm2=nm(Lh:end);
    
    prc1=prc(1:Lh);
    prc2=prc(Lh:end);
    % ind=(nm~=0);
    % nm=data(ind,1);
    % prc=data(ind,2);
    % prc(prc>5e6)=nan;
    
    h=figure(4)
    clf
    plot(nm,prc,'.-')
    plot(nm1,prc1,'r.-')
    hold on
    plot(nm2,prc2,'b.-')
    grid on
    legend('extend','withdraw','Location','NorthWest')
    xlabel('indent depth (nm)')
    ylabel('force (nN)')
  save_figure(h,[pfn '.png'])
    
    continue
    figure(5)
    clf
    % plot(nm,'r.-')
    % hold on
    plot(prc,'b.-')
    grid on
    
    [a,b]=ginput(2)
    a=round(a);
    ind=a(1):a(2);

    s_nm=nm(ind);
    s_prc=prc(ind);
    
    cfL=createFit_line_poly_N(s_nm,s_prc,1,1)
    %%
    kr=34.5380
    % kr=20
    PRC_sensitivity= 1.3073e+02
    
    kv=abs(cfL.p1);
    %%
    k_prc=kr*(PRC_sensitivity/kv-1)
    
    L0=134
    Lt=134%124+5
    kout(k)=k_prc/(L0/Lt)^3
end

std(kout)
mean(kout)