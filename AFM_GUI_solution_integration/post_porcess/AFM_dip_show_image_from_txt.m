% function AFM_dip_show_image_from_txt()
% while(1)
clear
% addpath('..\bin')
addpath('D:\AFMdata')
% while(1)
%     pause(3)
AFM_parameter=load('AFM_parameter.txt');

F={'AFM_HL.txt','AFM_HR.txt','AFM_EL.txt','AFM_ER.txt'};
figure(1)
clf
% point_now_x=AFM_parameter(1);
% point_now_y=AFM_parameter(2)
% point_now_y=abs(point_now_y);
point_now_x=1;
point_now_y=1

for k=1:length(F)
    f=F{k};
    ims{k}=load(f)';
end
AFM_dip_show_image(ims{1},ims{2},ims{3},ims{4},point_now_x,point_now_y,'');
% end