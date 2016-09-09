%%
function E_sample=fit_youngs_modulus(distance, force,para,rangeL,rangeH)
R=para.R;
v_sample=para.v_sample;
v_tip=para.v_tip;
E_tip=para.E_tip;

L=length(force);
ind=round([rangeL,rangeH].*L);
ind=ind(1):ind(2);
distance=distance(ind);
force=force(ind);

distance=distance-distance(1);
% cftool(distance, force)
cf= createFit_HertzModel(distance, force);
K=cf.K;
% K=K./(mag)^1.5

Ex=K/4*3/sqrt(R);
E_sample=(1-v_sample^2)./(1./Ex-(1-v_tip^2)./E_tip);
% E_sample=(1-v_sample^2).*Ex;
% E_out(k)=abs(E_sample)