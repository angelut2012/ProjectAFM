function [sx,sy,index]=manual_select_line_roi(x,y,str,varargin);
if nargin==5
indent_data_length=varargin{1};
AFM_type='brucker'
else
indent_data_length=length(y)-FindContactPoint_LineDistance(x,y);
AFM_type='PRC';
end


x=data_filter(x,indent_data_length);
y=data_filter(y,indent_data_length);
%% find the last noise point <0 as the start contact point
% inds=find(fy<0);
% inds=inds(end);

nx=normalize_01(x);
ny=(y)./max(y);

y2=abs((ny-min(ny))).^(2/3);
% y2=y2(1:sind);
x2=nx;%(1:sind);

% ind=find(ny>mean(ny)+std(ny)/2);
% ind=ind(1);
% L=length(ny)-ind;
% ind=ind-L;
% ind=max(1,ind);
% 
% ind2=find(ny<mean(ny)-std(ny)/2);
% if isempty(ind2)
%     ind2=round(length(ny)/4);
% end
% ind2=ind2(end);
% L=length(ny)-ind2;
% ind2=ind2-L;
% 
% ind= max(ind,ind2);

ind=max(1,length(y)-indent_data_length*5);


h=figure(500);
clf
plot(nx,'k.-')
hold on
plot(ny,'c.-')
plot(y2,'g.-')
grid on
legend('Z','tip','tip^{2/3}')
title(str)
xlim([ind length(ny)]);

global re_select_roi_N0_L1_C2
if re_select_roi_N0_L1_C2==-1
        a=auto_select_indent_ROI(ny)
%     if strcmp(AFM_type,'brucker')
%         a(1)=FindContactPoint_LineDistance(x2,y2);
%         a(2)=length(y);
%     end

A=a;
        a(1)=mean(A([1 1  1 1  2  2  2]));
        a(2)=mean(A([1 1 1 2 2 2 2]));
    a=round(a);
else
    [a,b]=ginput_judge_mat_save('data_manual_select_roi_contact.mat');
end
% a(1)=inds;
N=length(y);
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

function d=normalize_01(d);
mL=min(d);
mH=max(d);
d=(d-mL)./(mH-mL);
end
function y=data_filter(y,indent_data_length);
N=length(y);
% w=round(N/300);
w=ceil(indent_data_length/(31/3));
my=medfilt1(y,w);
W=ones(w,1)./w;
fy=filter2(W,my,'symmetric');
sind=length(y)-floor(w/2);
fy(sind:end)=y(sind:end);
y=fy;
end