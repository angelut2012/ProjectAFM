clear
clc
close all
clearvars -global
global re_select_roi
re_select_roi=1

%para.sensitivity_PRC_ReadoutPerNM=41.4506;%%110.602534853964968%279.83%167.5%[76.9865632572905];
para.probe_stiffness_nN_per_NM=30.005065977792331%29.899999559179317;

para.R=215/2;%nm
para.v_tip=0.15;
para.E_tip=135;%GPa
% % sio2
para.v_sample=0.17;
% %% bkr
para.v_sample=0.138;
para.v_sample=0.34;

% graphite
para.v_sample=0.17
%%%%%%%%%%%%%%%%%%%%%%%%%
pa=[]
filename='ch_p6*_extend.txt'
amp=1e9;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global pfn
FN=dir([pa filename])
for n=1:length(FN)
    %     n=2
    pfn=[pa FN(n).name]
    fn_store{n}=pfn;
    para.pfn=pfn;
    %% read data
    [z_piezo_NM,prc_readout,paras]=read_indentation_file(pfn);
    prc_readout=-prc_readout;
    
    
    z_piezo_NM=z_piezo_NM.*amp;
    prc_readout=prc_readout.*amp;
    
    %% show data in color
    show_indentation_data(z_piezo_NM,prc_readout);
    
    %% level_indentation_data
    [z_piezo_NM_c,prc_readout_adjusted_c]=level_indentation_data(z_piezo_NM,prc_readout);
    %% data format convert
    % use extend data
    
         %% convert and use extend data
    z_piezo_NM_extend=z_piezo_NM_c{1};
    Force=prc_readout_adjusted_c{1}; 
    [Displacement,Force]=convert_ZpiezoNMForce_to_DisplacementForce(z_piezo_NM_extend,Force,para.probe_stiffness_nN_per_NM);    
      %     %% select select indentation roi
    %     [sDisplacement,sForce,ind]=manual_select_curve_roi(Displacement,Force,'select indentation roi');
    %     %% calculate young's modulus
    %     Esample(n)=fit_youngs_modulus_linear_show_simulation(sDisplacement,sForce,para,0,1,para.fn)
    %% select select indentation roi
    %[sDisplacement,sForce,ind]=manual_select_curve_roi(Displacement,Force,'select indentation roi');
    [sDisplacement,sForce,ind]=manual_select_line_roi(Displacement,Force,'select indentation roi');
    %% calculate young's modulus
    Esample(n)=fit_youngs_modulus_linear_show_simulation(sDisplacement,sForce,para,0,1,para.pfn)
    
    
    
end