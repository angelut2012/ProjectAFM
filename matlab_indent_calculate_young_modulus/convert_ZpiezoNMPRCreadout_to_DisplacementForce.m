function [Displacement,Force]=convert_ZpiezoNMPRCreadout_to_DisplacementForce(z_piezo_NM,prc_readout,sensitivity_PRC_ReadoutPerNM,probe_stiffness_nN_per_NM);

% y=prc_readout;
% N=length(y);
% w=round(N/400);
% y=medfilt1(y,w);
% w=w;
% W=ones(w,1)./w;
% prc_readout=y;%filter2(W,y);


% z_tip_NM=prc_readout./sensitivity_PRC_ReadoutPerNM;
% Displacement_raw=z_piezo_NM-z_tip_NM;


% w=round(length(prc_readout)/200);
% prc_readout=medfilt1(prc_readout,w);
z_tip_NM=prc_readout./sensitivity_PRC_ReadoutPerNM;
Displacement=z_piezo_NM-z_tip_NM;
Force=z_tip_NM.*probe_stiffness_nN_per_NM;
% plot(indent_depth_NM,indent_force,'*-')
% 
% Displacement=indent_depth_NM(ind(1):ind(2));
% Force=indent_force(ind(1):ind(2));


end