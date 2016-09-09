clear
clc
close all
clearvars -global
global re_select_roi_N0_L1_C2
global select_extend1_withdraw2

re_select_roi_N0_L1_C2=2
select_extend1_withdraw2=1
pa='..\bin\'
pa=[]
FN=dir([pa 'ch_p66_extend.txt'])
amp=1e9;
for n=1:length(FN)
    %     n=2
    fn=[pa FN(n).name]
    fn_store{n}=fn;
    %% read data
    [z_piezo_NM,prc_readout,paras]=read_indentation_file(fn);
    prc_readout=-prc_readout;
    
    k=29.899999559179317;    
     paras{8}=1e6/2000;
     
    z_piezo_NM=z_piezo_NM.*amp;
    prc_readout=prc_readout.*amp;    

    prc_readout=prc_readout./k;

    
    %% show data in color
    show_indentation_data(z_piezo_NM,prc_readout);
    ylabel('force (nN)')
    ylabel('tip displacement (nm)')
    %% level_indentation_data
    [z_piezo_NM_c,prc_readout_adjusted_c]=level_indentation_data(z_piezo_NM,prc_readout);
    %%  calculate sensitivity and vibration noise
    [Sensitivity(n),noise_rms_nm(n)]=calculate_sensitivity_vibration_noise(z_piezo_NM_c{1},prc_readout_adjusted_c{1},paras);
end
disp('mean Sensitivity')
disp(mean(Sensitivity(n)))
noise_rms_nm
