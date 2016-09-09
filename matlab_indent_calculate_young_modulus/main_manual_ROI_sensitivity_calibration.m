clear
clc
close all
clearvars -global
global re_select_roi_N0_L1_C2
global select_extend1_withdraw2

re_select_roi_N0_L1_C2=2
select_extend1_withdraw2=2
pa='..\AFM_GUI_solution\bin\'
% filename= 'IndentData_SI_20150910164005.txt'
% pa='.\indentation_data\probe3_sensitivity_calibration_SiN_xy_coarse_fixed\'
%      filename='IndentData_*.txt'
% pa='.\indentation_data\Graphite_20150903\'
% pa='.\indentation_data\probe9_calibration_SI\'
filename='IndentData*.txt'
% pa='.\indentation_data\other\probe3_sensitivity_calibration_SiN_only_x_coarse_fixed\'
% filename='IndentData_AL_20150904004315.txt'



% pa='..\AFM_GUI_solution\bin\'
[filename,pa]=uigetfile([pa filename])


global pfn
FN=dir([pa filename])
for n=1:length(FN)
    %     n=2
    pfn=[pa FN(n).name]
    fn_store{n}=pfn;
    para.pfn=pfn;
    %% read data
    [z_piezo_NM,prc_readout,paras]=read_indentation_file(para.pfn);
%     prc_readout=-prc_readout;
    %% show data in color
    show_indentation_data(z_piezo_NM,prc_readout);
    
    %% level_indentation_data
    [z_piezo_NM_c,prc_readout_adjusted_c]=level_indentation_data(z_piezo_NM,prc_readout);
    %%  calculate sensitivity and vibration noise
    [Sensitivity(n),noise_rms_nm(n)]=calculate_sensitivity_vibration_noise(z_piezo_NM_c{1},prc_readout_adjusted_c{1},paras);
end
disp('mean Sensitivity')
disp(mean(Sensitivity(:)))
mean(noise_rms_nm)