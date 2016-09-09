function [Displacement,Force,Displacement_raw]=convert_ZpiezoNMPRCreadout_to_DisplacementForce_brucker(z_piezo_NM,prc_readout,sensitivity_PRC_ReadoutPerNM,probe_stiffness_nN_per_NM);

z_tip_NM=prc_readout./sensitivity_PRC_ReadoutPerNM;
Displacement=z_piezo_NM-z_tip_NM;

Force=z_tip_NM.*probe_stiffness_nN_per_NM;

global select_extend1_withdraw2
if select_extend1_withdraw2==2
    Displacement=-Displacement;
end
end