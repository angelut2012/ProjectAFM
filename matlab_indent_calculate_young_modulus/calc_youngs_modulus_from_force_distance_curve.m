function E_sample=calc_youngs_modulus_from_force_distance_curve(Force,Displacement,para)

%% select flat roi for adjusting leveling
[sx,sy,ind]=manual_select_curve_roi(z_piezo_NM,prc_readout,'select indentation roi');

z_tip_NM=prc_readout./para.sensitivity_PRC_ReadoutPerNM;
indent_depth_NM=z_piezo_NM-z_tip_NM;
indent_force=z_tip_NM*para.probe_stiffness_nN_per_NM;
% plot(indent_depth_NM,indent_force,'*-')

Displacement=indent_depth_NM(ind(1):ind(2));
Force=indent_force(ind(1):ind(2));

E_sample=fit_youngs_modulus_linear_show_simulation(Displacement,Force,para,0,1,para.fn)
