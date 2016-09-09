function [E_sample,EL,EH]=fit_youngs_modulus_linear_show_simulation(D,F,para,rangeL,rangeH,title_name)
% D=D-D(1);
% F=F-F(1);
% D=D-min(D);
% F=F-min(F);

[E_sample,EL,EH,cfL]=fit_youngs_modulus_linear(D,F,para,rangeL,rangeH)
% Eout(n)=fit_youngs_modulus(D,F,para)

N=1000;
D_sim=(0:N)./N.*(max(D));
F_sim=calc_force_distance_curve(D_sim,E_sample,para);
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
text(0.2,max(F_sim)*0.75,['Young''s modulus (Gpa):' num2str(E_sample)])
saveas(h,[para.pfn '_fit_modulus.tiff'])