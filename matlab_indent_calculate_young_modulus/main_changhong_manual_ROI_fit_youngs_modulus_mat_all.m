clear
clc
close all
clearvars -global
global re_select_roi_N0_L1_C2% -1 auto
global select_extend1_withdraw2
global show_figure_on1_off0
show_figure_on1_off0=1


re_select_roi_N0_L1_C2=-1
InMain_select_extend1_withdraw2=2


para.indent_data_length=100%50;
d_all=load('changhongAFM_all_20150925.mat')
para.sensitivity_PRC_ReadoutPerNM=1/(81.9);
%%110.602534853964968%279.83%167.5%[76.9865632572905];
para.probe_stiffness_nN_per_NM=25.3
%30.005065977792331%29.899999559179317;

para.R=130/2;%nm
para.v_tip=0.15;
para.E_tip=135;%GPa
% % sio2
% para.v_sample=0.17;
% % %% bkr
% para.v_sample=0.138;
% para.v_sample=0.34;

% graphite
% para.v_sample=0.17

%% PS
para.v_sample=0.34;
%Rtip(q);%200;%nm
%%teflon
% para.v_sample=0.46;
%% PDMS
% para.v_sample=0.5;
%%
para.v_tip=0.15;
para.E_tip=135;%GPa
%%%%%%%%%%%%%%%%%%%%%%%%%


pa=[]
% filename='ch_p6*_extend.txt'

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global pfn
% FN=dir([pa filename])
for n=5:148
    %     n=2
    select_extend1_withdraw2=InMain_select_extend1_withdraw2
%     n=78
    pfn=[num2str(n)]
    fn_store{n}=pfn;
    para.pfn=pfn;% try

    [z_piezo_NM,prc_readout]=read_indentation_file_Asylum(d_all,para,n);
    %% show data in color
    show_indentation_data(z_piezo_NM,prc_readout);
    
    %% level_indentation_data
    [z_piezo_NM_c,prc_readout_adjusted_c]=level_indentation_data(z_piezo_NM,prc_readout);
    %% convert and use extend data

%     [Displacement,Force]=convert_ZpiezoNMPRCreadout_to_DisplacementForce_brucker(z_piezo_NM_c{select_extend1_withdraw2},prc_readout_adjusted_c{select_extend1_withdraw2});
% Displacement=z_piezo_NM_c{select_extend1_withdraw2};
% Force=prc_readout_adjusted_c{select_extend1_withdraw2};
    [Displacement,Force]=convert_ZpiezoNMPRCreadout_to_DisplacementForce_brucker(z_piezo_NM_c{select_extend1_withdraw2},prc_readout_adjusted_c{select_extend1_withdraw2},para.sensitivity_PRC_ReadoutPerNM,para.probe_stiffness_nN_per_NM);
    %% select select indentation roi
    %[sDisplacement,sForce,ind]=manual_select_curve_roi(Displacement,Force,'select indentation roi');
    [sDisplacement,sForce,ind]=manual_select_line_roi(Displacement,Force,'select indentation roi',para.indent_data_length,'brucker');
%     sDisplacement=sDisplacement-min(sDisplacement);
    %% calculate young's modulus
    %fit_youngs_modulus_linear
    if InMain_select_extend1_withdraw2==2
        sDisplacement=-sDisplacement;
    end
    [Esample(n),EL(n),EH(n),cfL{n},gofR2(n)]=fit_youngs_modulus_linear(sDisplacement,sForce,para,0,1);
    
    show_ind=Displacement<max(sDisplacement);
    if show_figure_on1_off0==1
        [D_sim,F_sim,noise_rms_nN(n)]=show_simulation_force_distance_curve(cfL{n},Esample(n),para,Displacement(show_ind),Force(show_ind),para.pfn,gofR2(n));
    end% %     E(q,n)=Esample(n);
%     [Z_piezo,PRC]=convert_DisplacementForce_toZpiezoNMPRCreadout(D_sim,F_sim,para.sensitivity_PRC_ReadoutPerNM,para.probe_stiffness_nN_per_NM);
%     
%     %         [noise_rms_nm(q,n)]=calculate_vibration_noise_during_indentation(Displacement_raw(ind(1):ind(2)),500);
% n
%     catch
%         ind_err(n)=n;
%     end
end
Esample=Esample'
NumberOfPoints=8;
SamplePerPoints=21;



Q=reshape(Esample,SamplePerPoints,NumberOfPoints)
