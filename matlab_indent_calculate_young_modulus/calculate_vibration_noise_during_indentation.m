function [noise_rms_nm]=calculate_vibration_noise_during_indentation(sDisplacement,dt_us);
%% calculate sensitivity:
%select indentation curve ROI
N=length(sDisplacement);
dt_us=500;
t=(1:N).*dt_us;
cfC=createFit_line_poly_N(t,sDisplacement,1,1);

Q=sDisplacement-feval(cfC,t);
% t=t./(9.46);
figure(56)
plot(t,Q,'*-')
grid on
xlabel('time (us)')
ylabel('displacement (nm)')
title('vibration noise during indentation')

noise_rms_nm=std(Q);
end