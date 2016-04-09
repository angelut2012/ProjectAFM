function AFM_engage_calibration_TF_nm2V()
initial_global_define;
AFM_engage_set_gain_TF_V2NM(sys,-100);% initial gain

set(g_handles.edit_TF_sensitivity,'String','----');

AFM_engage_switch_TF_nm2V_calibration1_run0(sys,0);
pause(0.01)
AFM_engage_switch_TF_nm2V_calibration1_run0(sys,1);
step=1;% nm
N=20;
dz=0;
Z=zeros(N*4+2,1);
V=Z;
obj_V_delta_F= get_param([sys '/'  'scope_V_delta_F'], 'RuntimeObject');
r=[1:N N:-1:-N -N:0];
for k=1:length(r)
    dz=r(k)*step;
    engage_calibration_delta_z(sys,dz);
    pause(0.2)
    Z(k)=dz;
    V(k)=obj_V_delta_F.InputPort(1).Data;
end

engage_calibration_delta_z(sys,-N*step);
pause(5)
AFM_engage_switch_TF_nm2V_calibration1_run0(sys,0);
engage_calibration_delta_z(sys,0);
ft = createFit_TF_nm2V(V(21:60),Z(21:60));
TF_rate_V2nm=ft.p1;
TF_rate_V2nm=-abs(TF_rate_V2nm);
Gth=[100 400];
if abs(TF_rate_V2nm)>Gth(2)
        TF_rate_V2nm=sign(TF_rate_V2nm)*Gth(2);
end
if abs(TF_rate_V2nm)<Gth(1)
        TF_rate_V2nm=sign(TF_rate_V2nm)*Gth(1);
end
disp([ft.p1 ;TF_rate_V2nm])
AFM_engage_set_gain_TF_V2NM(sys,TF_rate_V2nm)
% Z
% V
% figure(1)
% plot(Z,V,'*-')
plot(g_handles.axes_plot_E,Z,V,'*-')


function ft = createFit_TF_nm2V(V,Z)
[xData, yData] = prepareCurveData( V,Z);
ft = fittype( 'poly1' );
opts = fitoptions( ft );
opts.Lower = [-Inf -Inf];
opts.Robust = 'Bisquare';
opts.Upper = [Inf Inf];
ft= fit( xData, yData, ft, opts );

function engage_calibration_delta_z(sys,delta_z)
set_param([sys '/'  'gain_TF_nm2V_delta_z'], 'Gain', num2str(delta_z));

