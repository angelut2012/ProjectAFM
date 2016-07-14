function out=para_evaluation(Vtr,Vtf);
std_tr=std(Vtr);
m_tr=mean(Vtr)
Vtr=mean(Vtr);% mean as true value

n=length(Vtf);
nc=3;
acc_nm=1/n*sqrt(sum((Vtf-Vtr).^2))*nc;
acc_p=acc_nm/Vtr*100;
reap=std(Vtf)*nc;
m_tf=mean(Vtf);
std_tf=std(Vtf);

err=m_tf-m_tr;
acc_cum=abs(err)+reap/nc;
out=[acc_p acc_nm reap m_tf std_tf m_tr std_tr err acc_cum];
end