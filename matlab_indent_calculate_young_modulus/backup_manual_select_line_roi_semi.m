function [sx,sy,index]=manual_select_line_roi_semi(x,y,str);

N=length(y);
w=round(N/200);
y=medfilt1(y,w);
w=w;
W=ones(w,1)./w;
fy=filter2(W,y);
%% find the last noise point <0 as the start contact point
inds=find(fy<0);
inds=inds(end);



nx=normal_scale(x);
ny=(y)./max(y);

% % n=1:N-1;
% dy=diff(ny);
% ndy=normal_scale(dy);
% % 
% 
% n=1:length(ny);
% cfS=createFit_FFT8(n,ny);
% % cfS=createFit_line_poly_N(n,ny,8);
% fy=feval(cfS,n);
% dfy=diff(fy,2);
% % ind=find()

h=figure(500);
clf
plot(nx,'k.-')
hold on
plot(ny,'c.-')
grid on
title(str)
dfn='data_manual_select_roi_contact.mat';
[a,b]=ginput_judge_mat_save(dfn);
a(1)=inds;
a(a>N)=N;
a(a<1)=1;
% ind=x>a(1)&x<a(2);
ind=a(1):a(2);
sx=x(ind);
sy=y(ind);
% inds=find(ind==1);
index=a;
close(h)
end

function d=normal_scale(d);
mL=min(d);
mH=max(d);
d=(d-mL)./(mH-mL);
end
