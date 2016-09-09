clear
load ch_d.mat
amp=1e9;
ind=2880:3121;
d=d.*1e9;
Force=Force.*amp;
d=d(ind,:);
Force=Force(ind);


z_sample_depth_nm=d(:,6);
z_piezo_nm=d(:,1);

z_sample_depth_nm=z_sample_depth_nm-z_sample_depth_nm(1);
cfC=createFit_line_poly_N(z_sample_depth_nm,Force,1,1)
a=1/29.256040773715274-1/1.171962477211917e+03
k=1/a=30.005065977792331