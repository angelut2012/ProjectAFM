clear
close all

pa=('C:\AFMdata\')
% pa='C:\AFMdata\AFM_v3_zeros_scan\coarse_on_laser_on'
fn= '*Image_ZeroScanSi_XYoff_Coarse_PowerOff_test2_128p_2Hz_1nm_3s_HR*.txt'
fn='Image_ZeroScanSi1_128p_1Hz_1nm_3s_HR*.txt'
fn='Image_ZeroScanSi_XYoff_Coarse_PowerOnLaserOff_128p_2Hz_1nm_3s_HR*.txt'
fn='Image_ZeroScanSi_XYoff_Coarse_PowerOnLaserOn_128p_2Hz_1nm_3s_HR*.txt'

fn='Image_FlatScanSi_Coarse_PowerOn_JustAfterLaserOff_128p_1Hz_1000nm_3s_HR*.txt'

if pa(end)~='\'
    pa=[pa '\'];
end


[fn,pa]=uigetfile([pa fn])


fns=dir([pa fn])
for k=1:length(fns)
    fn=fns(k).name
    pfn=[pa fn]
    
    imraw=load(pfn);
    % imraw=medfilt2(imraw,[3,3],'symmetric');
    
    imraw(imraw==-1)=nan;
    imraw=-imraw;
    % imraw=imraw-min(imraw(:));
    % imraw=imraw-( -7.526316248554960e+03+1100)
    figure(1)
    surf(imraw,'LineStyle','non')
    colorbar
    zlabel('nm')
%     ylabel('y<<--')
        ylabel('y ')%-->>
    view([-112,18])
    % title(fn)
    
    
    saveas(gcf,[ pfn '_1.tiff'])
    
    L=mean(imraw,2);
    L=mean(imraw(:,1:20),2);
    % L=flipud(L);
    figure(2)
    plot(L,'*-')
    ylabel('nm')
    xlabel('y ')%-->>
    
    saveas(gcf,[ pfn '_2.tiff'])
    % title(fn)
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
    load point_select.mat
    im_flat=image_adjust_plane(imraw,1,px,py);
    % im_flat=image_adjust_plane2(imraw,1,1);%
    % im_flat=medfilt2(im_flat,[3,3],'symmetric');
    im_flat=im_flat-min(im_flat(:));
    im_flat=im_flat-mean(im_flat(:));
    figure(3)
    surf(im_flat,'LineStyle','non')
    colorbar
    zlabel('nm')
    % zlim([-10 10])
    
    saveas(gcf,[ pfn '_3.tiff'])
    
    figure(4)
    imshow(im_flat,[])
    
    set(gca,'LooseInset',get(gca,'TightInset'))
    colorbar
    title(fn)
    
    
    saveas(gcf,[ pfn '_4.tiff'])
    
    % [x,y]=ginput(1);
    % y=round(y)
    % L=im_flat(y,:);
    % figure(3)
    % plot(L,'.-')
    
    p=im_flat(:);
    figure(5)
    hist(p,min(p):0.1:max(p))
    title([fn ' std=' num2str(std(p))],'Interpreter','non')
    xlabel('noise (nm)')
    ylabel('point count')
    
    % [image_show_buffer,im_buffer]=AFM_dip_for_show(im_flat,127,127);
    std(p)
    saveas(gcf,[ pfn '_5.tiff'])
end
return

%% 

% 
% 
% if ~exist('point_select.mat','file')
%     
% else
%     load point_select.mat
%     im_flat=image_adjust_plane(imy,px,py,1);
% end
% 
% 
% im_flat(im_flat<-20)=0;
% 
% %     mfn=5;
% %     im_flat=medfilt2(im_flat,[mfn mfn], 'symmetric');
% %
% 
% % %     im=im_flat(:);
% % %     [p,ind]=hist(im,min(im):0.5:max(im));
% % %     p(p<mean(p))=0;
% % %     [pks,locs]=findpeaks(p,'minpeakdistance',200);
% % %     H(k)=diff(ind(locs));
% 
% % figure(1)
% % imshow(im,[])
% % figure(2)
% % imshow(imy,[])
% fh=figure(3)
% im_flat_s=imresize(im_flat,3);
% imshow(im_flat_s,[ ])
% %     set(fh,'Position',[  111   112   814   713])
% %     get(fh,'Position')
% %     title(num2str(k))
% title( f(k).name)
% 
% 
% % figure(4)
% %     surf(im_flat)
% %     pause(0.1)
% 
% IM_flat{k}=im_flat;
% %     [dy(k),dx(k),dp(k)]= dftregistration(IM_flat{1},IM_flat{k},1);
% %     text(0,800,['frame: ' num2str(k) '   dx: ' num2str(dx(k)) '   dy: ' num2str(dy(k))],'FontSize',12)
% 
% if show_onoff==1
%     pause(0.5)
%     saveas(fh,['im' num2str(k) '.tif'])
%     gim(k)=getframe;
% end
% 
% 
% if show_onoff==1
%     fh4=figure(4)
%     surf(im_flat)
%     %         axis([1 128 1 128 -250 100])
%     colormap default
%     view([23.5 54])
%     title( f(k).name)
%     pause(0.5)
%     saveas(fh4,['surf' num2str(k) '.tif'])
%     %         gsurf(k)=getframe;
% end
% 
% %     vidObj = VideoWriter('v1.mp4');
% %     open(vidObj);
% % for k=s
% %   writeVideo(vidObj,gim(k) );
% % end
% % for k=s
% %   writeVideo(vidObj, gsurf(k) );
% % end
% %   % Close the file.
% %     close(vidObj);
% 
% 
% 
% 
% %
% % H(H==0)=[];
% % figure
% % plot(H,'*-')
% % hs=H;
% % plot(hs,'*-')
% % mean(hs)
% % std(hs)
% %
% %
% %
% % figure
% %
% % hist(H,min(H):0.5:max(H))
% % [h,ind]=sort(H)
% % hs=h(15:25)
% % ind(15:25)
% % plot(hs,'*-')
% % mean(hs)
% % std(hs)
% 
% %
% % %%%%%%%%%%%%%%%%
% %
% % % for k=1:length(f)
% % sx=44;
% % sy=51;
% % % h=imrect
% % % wait(h)
% % %
% % % PO=[ 37.000000000000014  11.000000000000000  48.000000000000000  55.000000000000000]
% % %  sx=PO(1):PO(1)+PO(3);
% % % sx=PO(2):PO(2)+PO(4);
% %
% % imshow(im_flat,[-250 50])
% % [sy,sx]=ginput(1)
% % sx=round(sx)
% % sy=round(sy)
% % for k=s
% %
% %     [dy(k),dx(k),dp(k)]= dftregistration(IM_flat{16},IM_flat{k},1);
% %
% %     P(k)=IM_flat{k}(sy+dy(k),sx+dx(k));
% %
% %     IM_flat_re{k}=imfftreconstruct(IM_flat{k},-dy(k),-dx(k),-dp(k));
% %     imshow(IM_flat_re{k},[])
% %     % pause(0.5)
% % end
% % P(P==0)=[];
% % figure
% % plot(P,'*-')
% % hs=P;
% % M=mean(hs)
% % STD=std(hs)
% % err_P=max(P)-mean(P)
% % err_N=min(P)-mean(P)
% % figure
% % hist(hs,min(hs):1:max(hs))
% % return
% % %  H=P;
% %
% %
% % %