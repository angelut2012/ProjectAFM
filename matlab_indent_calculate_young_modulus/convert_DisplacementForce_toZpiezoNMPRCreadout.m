function [Z_piezo,PRC]=convert_DisplacementForce_toZpiezoNMPRCreadout...
    (D,F,sensitivity_PRC_ReadoutPerNM,probe_stiffness_nN_per_NM);
% D=D(end);
d_tip=F./probe_stiffness_nN_per_NM;
PRC=d_tip.*sensitivity_PRC_ReadoutPerNM;
Z_piezo=D+d_tip;
end
