function [E_sample,EL,EH,cfL]=fit_youngs_modulus_linear_pyramid3(distance, force,para,rangeL,rangeH)
R=para.R;
v_sample=para.v_sample;
v_tip=para.v_tip;
E_tip=para.E_tip;

L=length(force);
% ind=round([0.1 0.9].*L);
ind=round([rangeL rangeH].*L);
if ind(1)==0
    ind(1)=1;
end
ind=ind(1):ind(2);
distance=distance(ind);
force=force(ind);

% distance=distance-distance(1);
% cftool(distance, force)
%% linearized hertz model
order=2
force(force<0)=0;
L_force=force.^(1/order);
[cfL,fh]=createFit_line_poly_N(distance, L_force,1,54);
K=cfL.p1^(order);
%% hertz model
% cf= createFit_HertzModel(distance, force);
% K=cf.K
% K=K./(mag)^1.5
th=atan(136/203)/2;
part=4*tan(th)/(3*sqrt(3));
Ex=K/part;
E_sample=(1-v_sample^2)./(1./Ex-(1-v_tip^2)./E_tip);
%%
Kc=confint(cfL).^(order);;
Kc=Kc(:,1);
Ec=Kc./part;
E_conf=(1-v_sample^2)./(1./Ec-(1-v_tip^2)./E_tip);
EL=E_conf(1);
EH=E_conf(2);
% E_sample=(1-v_sample^2).*Ex;
% E_out(k)=abs(E_sample)

title('fit youngs modulus linear')
grid on
legend('experimental data','curve fit')
% title(title_name,'Interpreter','non')
xlabel( 'indent depth (nm)' );
ylabel( 'force.^{(2/3)} (nN)' );
end