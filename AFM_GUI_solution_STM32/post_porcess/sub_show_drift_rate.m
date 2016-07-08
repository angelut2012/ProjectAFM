function sub_show_drift_rate(t,w)

cf=createFit_line_poly_N(t,w,6,1)
ylabel('sensor readout (nm)')
xlabel('time (minute)')
dcf=cf;
dcf.p4=dcf.p3;
dcf.p3=dcf.p2;
dcf.p2=dcf.p1;
dcf.p1=0;
rate=feval(dcf,t);

tn=linspace(t(1),t(end),length(t));
wn=feval(cf,tn);
dwn=diff(wn);
tn=tn(1:end-1);

% title (['drift rate = ' num2str(cf.p1)  ' nm/min'])
figure(2)
% plot(t,rate)
plot(tn,dwn)
ylabel('sensor readout drift rate (nm/min)')
xlabel('time (minute)')
grid on
end