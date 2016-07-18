%% 20160428
clear
% load input.mat
close all
% load('input_20150827031119.mat')
%     imy=image_adjust_Y(imHL',123,128);
p=('C:\AFMdata\')
[fn,p]=uigetfile([p '\*.txt'])
imraw=load([p fn]);
% imraw=medfilt2(imraw,[3,3],'symmetric');

% imraw(imraw==-1)=nan;
imraw=-imraw;
% imraw=imraw-min(imraw(:));
imraw=imraw-( -7.526316248554960e+03)
% figure(10)
% surf(imraw,'LineStyle','non')
% colorbar
% zlabel('nm')
% ylabel('y<<--')
% view([-82,14])
% title(fn)


L=mean(imraw,2);
% L=mean(imraw(:,1:20),2);
L=flipud(L);
figure(20)
plot(L)
ylabel('nm')
title(fn)
% return

% imraw(1:30,:)=[];
% imraw=imraw';
% imm=mean(imraw(:))-imraw;
% imm=imraw-min(imraw(:));
% surf(imm)
% return

% imL=image_adjust_each_line(imm,1);
% figure(1)
% imshow(imL,[])
% [x,y]=ginput(1)
% imy=image_adjust_Y(imL,15,16);
% imf=medfilt2(imm,[3,3]);
%     imy=im;
ps=linspace(1,size(imraw,1),20);
[px,py]=meshgrid(ps,ps);
px=px(:);
py=py(:);
im_flat=imraw;
% im_flat=image_adjust_plane(imraw,1,px,py);
im_flat=image_adjust_plane2(imraw,2,2);%
% im_flat=medfilt2(im_flat,[3,3],'symmetric');
im_flat=im_flat-min(im_flat(:));
im_flat=im_flat-mean(im_flat(:));
figure(1)
surf(im_flat,'LineStyle','non')
colorbar
zlabel('nm')
% zlim([-10 10])
figure(2)
imshow(im_flat,[])

set(gca,'LooseInset',get(gca,'TightInset'))
colorbar 
% title(fn)

% [x,y]=ginput(1);
% y=round(y)
% L=im_flat(y,:);
% figure(3)
% plot(L,'.-')

p=im_flat(:);
figure(4)
hist(p,min(p):0.1:max(p))
title([fn ' std=' num2str(std(p))],'Interpreter','non')
xlabel('noise (nm)')
ylabel('point count')

% [image_show_buffer,im_buffer]=AFM_dip_for_show(im_flat,127,127);
std(p)


return

if ~exist('point_select.mat','file')
    
else
    load point_select.mat
    im_flat=image_adjust_plane(imy,px,py,1);
end


im_flat(im_flat<-20)=0;

%     mfn=5;
%     im_flat=medfilt2(im_flat,[mfn mfn], 'symmetric');
%

% %     im=im_flat(:);
% %     [p,ind]=hist(im,min(im):0.5:max(im));
% %     p(p<mean(p))=0;
% %     [pks,locs]=findpeaks(p,'minpeakdistance',200);
% %     H(k)=diff(ind(locs));

% figure(1)
% imshow(im,[])
% figure(2)
% imshow(imy,[])
fh=figure(3)
im_flat_s=imresize(im_flat,3);
imshow(im_flat_s,[ ])
%     set(fh,'Position',[  111   112   814   713])
%     get(fh,'Position')
%     title(num2str(k))
title( f(k).name)


% figure(4)
%     surf(im_flat)
%     pause(0.1)

IM_flat{k}=im_flat;
%     [dy(k),dx(k),dp(k)]= dftregistration(IM_flat{1},IM_flat{k},1);
%     text(0,800,['frame: ' num2str(k) '   dx: ' num2str(dx(k)) '   dy: ' num2str(dy(k))],'FontSize',12)

if show_onoff==1
    pause(0.5)
    saveas(fh,['im' num2str(k) '.tif'])
    gim(k)=getframe;
end


if show_onoff==1
    fh4=figure(4)
    surf(im_flat)
    %         axis([1 128 1 128 -250 100])
    colormap default
    view([23.5 54])
    title( f(k).name)
    pause(0.5)
    saveas(fh4,['surf' num2str(k) '.tif'])
    %         gsurf(k)=getframe;
end

%     vidObj = VideoWriter('v1.mp4');
%     open(vidObj);
% for k=s
%   writeVideo(vidObj,gim(k) );
% end
% for k=s
%   writeVideo(vidObj, gsurf(k) );
% end
%   % Close the file.
%     close(vidObj);




%
% H(H==0)=[];
% figure
% plot(H,'*-')
% hs=H;
% plot(hs,'*-')
% mean(hs)
% std(hs)
%
%
%
% figure
%
% hist(H,min(H):0.5:max(H))
% [h,ind]=sort(H)
% hs=h(15:25)
% ind(15:25)
% plot(hs,'*-')
% mean(hs)
% std(hs)

%
% %%%%%%%%%%%%%%%%
%
% % for k=1:length(f)
% sx=44;
% sy=51;
% % h=imrect
% % wait(h)
% %
% % PO=[ 37.000000000000014  11.000000000000000  48.000000000000000  55.000000000000000]
% %  sx=PO(1):PO(1)+PO(3);
% % sx=PO(2):PO(2)+PO(4);
%
% imshow(im_flat,[-250 50])
% [sy,sx]=ginput(1)
% sx=round(sx)
% sy=round(sy)
% for k=s
%
%     [dy(k),dx(k),dp(k)]= dftregistration(IM_flat{16},IM_flat{k},1);
%
%     P(k)=IM_flat{k}(sy+dy(k),sx+dx(k));
%
%     IM_flat_re{k}=imfftreconstruct(IM_flat{k},-dy(k),-dx(k),-dp(k));
%     imshow(IM_flat_re{k},[])
%     % pause(0.5)
% end
% P(P==0)=[];
% figure
% plot(P,'*-')
% hs=P;
% M=mean(hs)
% STD=std(hs)
% err_P=max(P)-mean(P)
% err_N=min(P)-mean(P)
% figure
% hist(hs,min(hs):1:max(hs))
% return
% %  H=P;
%
%
% %