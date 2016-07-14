clear
close all
show_onoff=0;
load im_grid
q=1

Y=image_adjust_Y(im',1,3)
% y=image_adjust_fit_Y(Y,2);

        imy=Y;
        
        if ~exist('point_select.mat','file')
            im_flat=image_adjust_plane(imy);
        else
            load point_select.mat
            im_flat=image_adjust_plane(imy,px,py,2);
        end
                
   L=  im_flat(:,100);
   figure(3)
   plot(L)
     
     
     
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
        imshow(im_flat_s,[])
        set(fh,'Position',[  111   112   814   713])
        %     get(fh,'Position')
        %     title(num2str(k))
        title( f(k).name)
        
        
        % figure(4)
        %     surf(im_flat)
        %     pause(0.1)

        %     [dy(k),dx(k),dp(k)]= dftregistration(IM_flat{1},IM_flat{k},1);
        %     text(0,800,['frame: ' num2str(k) '   dx: ' num2str(dx(k)) '   dy: ' num2str(dy(k))],'FontSize',12)
        
        if show_onoff==1
            pause(0.5)
            saveas(fh,['im' num2str(k) '.tif'])
            gim(k)=getframe;
        end
        %     fh4=figure(4)
        %     surf(im_flat)
        
        if show_onoff==1
            
            axis([1 128 1 128 -250 100])
            colormap default
            view([-23.5 54])
            title( f(k).name)
            pause(0.5)
            saveas(fh4,['surf' num2str(k) '.tif'])
            gsurf(k)=getframe;
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