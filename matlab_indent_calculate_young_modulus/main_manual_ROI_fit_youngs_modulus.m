clear
clc
close all
clearvars -global
global re_select_roi_N0_L1_C2% -1 auto
global select_extend1_withdraw2
global show_figure_on1_off0
show_figure_on1_off0=1


re_select_roi_N0_L1_C2=1
InMain_select_extend1_withdraw2=1

para.sensitivity_PRC_ReadoutPerNM=47.3%38.3%64.1638%36.2446
%20.1536%37.1186% probe 9
para.probe_stiffness_nN_per_NM= 58.841% 44.7%117.3131%40;
para.R=16.9412e3%53.4730%9.6051e3%62%probe3
%% tip material
%% silicon tip
para.v_tip=0.15;
para.E_tip=135;%GPa

% %%borosilicate ball tip
% para.v_tip=0.2;
% para.E_tip=60;%GPa


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

% load PRC_needle_PS_new_stiffness



%%%%%%%%%%%%%%%%%%%%%%%%%
pa='..\AFM_GUI_solution\bin\'
% pa='..\AFM_GUI_solution\bin\PTFE_large_force\'
% pa='.\indentation_data\PS_probe9\'
% pa='.\indentation_data\PTFE_large_force\'
% pa='.\indentation_data\Graphite_20150903\'
% pa='.\indentation_data\PDMSN\'

% pa='.\indentation_data\ball_PS\'
% filename='IndentData_PS*.txt'
% pa='F:\google_drive\AFMdata\indentation_data\needle_PS\'
% filename='IndentData_PS*.txt'

pa='D:\AFMdata\'
filename=''
[filename,b]=uigetfile([pa filename])
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global pfn
FN=dir([pa filename])
for n=1:length(FN)
    select_extend1_withdraw2=InMain_select_extend1_withdraw2
    %     n=23
    pfn=[pa FN(n).name]
    fn_store{n}=pfn;
    para.pfn=pfn;
    %% read data
    [z_piezo_NM,prc_readout,paras]=read_indentation_file(pfn);
    %% show data in color
    prc_readout=prc_readout-mean(prc_readout(1:length(prc_readout)/4));
    prc_readout_nm=prc_readout./para.sensitivity_PRC_ReadoutPerNM;
    show_indentation_data(z_piezo_NM,prc_readout_nm);
    
    %% level_indentation_data
    [z_piezo_NM_c,prc_readout_adjusted_c]=level_indentation_data(z_piezo_NM,prc_readout);
    %% convert and use extend data
    [Displacement,Force]=convert_ZpiezoNMPRCreadout_to_DisplacementForce(z_piezo_NM_c{select_extend1_withdraw2},prc_readout_adjusted_c{select_extend1_withdraw2},para.sensitivity_PRC_ReadoutPerNM,para.probe_stiffness_nN_per_NM);
    %% select select indentation roi
    %[sDisplacement,sForce,ind]=manual_select_curve_roi(Displacement,Force,'select indentation roi');
    [sDisplacement,sForce,ind]=manual_select_line_roi(Displacement,Force,'select indentation roi');
    %     sDisplacement=sDisplacement-min(sDisplacement);
    %% calculate young's modulus
    %fit_youngs_modulus_linear
    [Esample(n),EL(n),EH(n),cfL{n},gofR2(n)]=fit_youngs_modulus_linear(sDisplacement,sForce,para,0,1);
    
    disp([n gofR2(n)])
%     redo=4;
    if(gofR2(n)>1)
%         try
%             ind(1)=ind(1)-(ind(2)-ind(1))*redo;
%             ind(1)=max(floor(length(Force)/2),ind(1));
%             inds=ind(1);
%             [sDisplacement,sForce,ind]=manual_select_line_roi(Displacement(ind(1):ind(2)),Force(ind(1):ind(2)),'select indentation roi');
%             ind=inds+ind;
%             [Esample(n),EL(n),EH(n),cfL{n},gofR2(n)]=fit_youngs_modulus_linear(sDisplacement,sForce,para,0,1);
%             disp('retry')
%             disp(gofR2(n))
%             redo=redo-1;
%             if redo<0
%                 break
%             end
%             
%         catch
            T=re_select_roi_N0_L1_C2;
            re_select_roi_N0_L1_C2=2;
            [sDisplacement,sForce,ind]=manual_select_line_roi(Displacement,Force,'select indentation roi');
%             ind=inds+ind;
            [Esample(n),EL(n),EH(n),cfL{n},gofR2(n)]=fit_youngs_modulus_linear(sDisplacement,sForce,para,0,1);
            re_select_roi_N0_L1_C2=T;
            disp('manual')
            disp(gofR2(n))
%             break
%         end
    end
    
    show_ind=Displacement<max(sDisplacement);
    if show_figure_on1_off0==1
        [D_sim,F_sim,noise_rms_nN(n)]=show_simulation_force_distance_curve(cfL{n},Esample(n),para,Displacement(show_ind),Force(show_ind),para.pfn,gofR2(n));
    end
    % %     E(q,n)=Esample(n);
    %     [Z_piezo,PRC]=convert_DisplacementForce_toZpiezoNMPRCreadout(D_sim,F_sim,para.sensitivity_PRC_ReadoutPerNM,para.probe_stiffness_nN_per_NM);
    %
    %     %         [noise_rms_nm(q,n)]=calculate_vibration_noise_during_indentation(Displacement_raw(ind(1):ind(2)),500);
    % n
end

Esample=Esample';

NumberOfPoints=23;
SamplePerPoints=5;
Q=reshape(Esample,SamplePerPoints,NumberOfPoints)

clc
for n=1:length(FN)
    fprintf('%s\n',fn_store{n})
end
% for n=1:length(FN)
% fprintf('%0.3f\n',gofR2(n).rsquare)
% end
% for n=1:length(FNS)
%     fprintf('%s\n',FNS{n})
% end
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