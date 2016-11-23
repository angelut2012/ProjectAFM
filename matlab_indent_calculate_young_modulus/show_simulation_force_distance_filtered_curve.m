function [D_sim,F_sim,noise_rms]=show_simulation_force_distance_filtered_curve(cfL,E_sample,para,Displacement,Force,title_name,gof)
d0=-cfL.p2/cfL.p1;

ind=find(Displacement>d0);
% D=Displacement(ind)-d0;
% F=Force(ind);

D=Displacement;
D=D-d0;
% D=D-D(1);
F=Force;
F=F-F(1);


N=1000;
D_sim=(0:N)./N.*(max(D(1:end-5)));
% D_sim=(0:N)./N.*(max(D(1:end-10)));
F_sim=calc_force_distance_curve(D_sim,E_sample,para);
% F_sim=calc_force_distance_curve_pyramid3(D_sim,E_sample,para);
h=figure(55);
clf
plot(D,F,'b.-')
hold on
plot(D_sim,F_sim,'r-')
grid on
legend('experimental data','simulated curve',2)
title(title_name,'Interpreter','non')
xlabel( 'indent depth (nm)' );
ylabel( 'force (nN)' );
text(0.2,max(F_sim)*0.75,['Young''s modulus (Gpa):' num2str(E_sample) ', R^2=' num2str(gof)])
saveas(h,[para.pfn '_fit_modulus.tiff'])
return
%% noise 
noise=Force(ind(1:end-10))-calc_force_distance_curve(D(1:end-10),E_sample,para);
noise_rms=std(noise);
end