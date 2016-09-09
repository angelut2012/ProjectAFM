clear
clc
warning off
close all
clearvars -global
global re_select_roi_N0_L1_C2% -1 auto
global select_extend1_withdraw2
global show_figure_on1_off0
show_figure_on1_off0=0


re_select_roi_N0_L1_C2=-1
InMain_select_extend1_withdraw2=2

para.indent_data_length=1300%50;
para.sensitivity_PRC_ReadoutPerNM=1% brucker
para.probe_stiffness_nN_per_NM= 96.45%ball;20.98%bprobe7 needle
para.R=16.1e3%31.75%62%probe3
%% tip material
% %% silicon tip
% para.v_tip=0.15;
% para.E_tip=135000;%GPa

%%borosilicate ball tip
para.v_tip=0.2;
para.E_tip=60;%GPa


% % sio2
%     para.v_sample=0.17;
% %% bkr
%     para.v_sample=0.138;
% graphite
% para.v_sample=0.17;% 0.17~0.23
% %% ptfe
%     para.v_sample=0.46;
%% PS
para.v_sample=0.34;
%Rtip(q);%200;%nm

%% PDMS
% para.v_sample=0.5;





%%%%%%%%%%%%%%%%%%%%%%%%%
% pa='..\AFM_GUI_solution\bin\PTFE_large_force\'
%     pa='..\AFM_GUI_solution\bin\'
% pa='.\indentation_data\PS_probe9\'
% pa='.\indentation_data\PTFE_large_force\'
% pa='.\indentation_data\Graphite_20150903\'
% pa='.\indentation_data\PDMSN\'

% pa='\ball_PS\'
% filename='IndentData_PS*.txt'
% pa='.\indentation_data\ball_teflon\'
% pa='F:\google_drive\project\ball_indent_burcker\bprobe5_ball_indentation\indent_trigger_200nm\'
% pa='.\indentation_data\brucker_bprobe7_needle\'

% pa='F:\brucker_bprobe7_needle_PS\'
% pa='.\indentation_data\brucker_bprobe7_needle_PS\'
pa='.\indentation_data\brucker_bporbe5_ball_PS\'
filename='*.txt'
% [filename,b]=uigetfile([pa filename])

% load E_brucker_bprobe7_needle_PS_dis1um_R2
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global pfn
FN=dir([pa filename])
for n=1:length(FN)
    try
    select_extend1_withdraw2=InMain_select_extend1_withdraw2
%     n=78
    pfn=[pa FN(n).name]
    fn_store{n}=pfn;
    para.pfn=pfn;
    %% read data
    [z_piezo_NM,prc_readout,paras]=read_indentation_file_brucker(pfn,para.indent_data_length);
  

    
    
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
%     if InMain_select_extend1_withdraw2==2
%         sDisplacement=-sDisplacement;
%     end
    [Esample(n),EL(n),EH(n),cfL{n},gofR2(n)]=fit_youngs_modulus_linear(sDisplacement,sForce,para,0,1);
    
    show_ind=Displacement<max(sDisplacement);
    if show_figure_on1_off0==1
        [D_sim,F_sim,noise_rms_nN(n)]=show_simulation_force_distance_curve(cfL{n},Esample(n),para,Displacement(show_ind),Force(show_ind),para.pfn,gofR2(n));
    end% %     E(q,n)=Esample(n);
%     [Z_piezo,PRC]=convert_DisplacementForce_toZpiezoNMPRCreadout(D_sim,F_sim,para.sensitivity_PRC_ReadoutPerNM,para.probe_stiffness_nN_per_NM);
%     
%     %         [noise_rms_nm(q,n)]=calculate_vibration_noise_during_indentation(Displacement_raw(ind(1):ind(2)),500);
% n
    catch
        ind_err(n)=n;
    end
end
Esample=Esample'
NumberOfPoints=7;
SamplePerPoints=11;



Q=reshape(Esample,SamplePerPoints,NumberOfPoints)
return
clc
for n=1:length(FN)
fprintf('%s\n',fn_store{n})
end

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