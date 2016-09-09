function [Displacement,Force]=convert_ZpiezoNMForce_to_DisplacementForce(z_piezo_NM,Force,probe_stiffness_nN_per_NM);
% w=round(length(prc_readout)/200);
% prc_readout=medfilt1(prc_readout,w);
z_tip_NM=Force./probe_stiffness_nN_per_NM;
Displacement=z_piezo_NM-z_tip_NM;
end