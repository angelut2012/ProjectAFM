function force=calc_force_distance_curve_pyramid3(D,E_sample,para)
Ex=1/((1-para.v_tip^2)/para.E_tip+(1-para.v_sample^2)/E_sample);

th=atan(136/203)/2;
part=4*tan(th)/(3*sqrt(3));

force=part.*Ex.*D.^2;
% force=4/3*Ex*sqrt(para.R).*D.^1.5;

end
