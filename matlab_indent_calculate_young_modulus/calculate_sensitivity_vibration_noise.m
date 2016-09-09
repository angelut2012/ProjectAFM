function [Sensitivity,noise_rms_nm]=calculate_sensitivity_vibration_noise(z_piezo_NM,prc_readout_adjusted,paras);
%% calculate sensitivity:
%select indentation curve ROI
[sx,sy,ind]=manual_select_line_roi(z_piezo_NM,prc_readout_adjusted,'select indentation curve ROI');

% sx=z_piezo_NM;
% sy=prc_readout_adjusted;

prc_readout_b18=sy-min(sy);
displacement_NM=sx;
displacement_NM=displacement_NM-min(displacement_NM);
cfC=createFit_line_poly_N(displacement_NM,prc_readout_b18,1,1);
grid on
xlabel('displacement of Z actuator (nm)')
ylabel('sensor readout Bit18')
title(['Sensitivity=' num2str(cfC.p1) '/nm'])
Sensitivity=cfC.p1;
% Sensitivity=36.2446;
% Sensitivity=56.38535899452257

%% calculate sensitivity:
%show vibration

% Sensitivity=paras{6}
Sensitivity=38
figure(51)
Q=prc_readout_b18-feval(cfC,displacement_NM);
Qnm=Q./Sensitivity;
dt_us=paras{8};
t=(1:length(Q)).*dt_us;%./length(Q);
% t=t./(9.46);
plot(t,Qnm,'*-')
grid on
xlabel('time (us)')
ylabel('displacement (nm)')
title('vibration noise during indentation')

noise_rms_nm=std(Qnm);
end