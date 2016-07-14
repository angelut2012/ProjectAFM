function [image_show_buffer,im_buffer]=AFM_dip_for_show(im_buffer,point_now_x,point_now_y,show_line)
% im_buffer(im_buffer==-1)=nan;

im_buffer=double(im_buffer);
if point_now_y<0% select image range to process
    imsf=flipud(im_buffer);
else
    imsf=im_buffer;
end
point_now_x=abs(point_now_x);
point_now_y=abs(point_now_y);
[N_x,N_y]=size(im_buffer);
len=point_now_x+point_now_y*N_x;

r_adj=0;
if len>N_y*N_x*0.1
    rind=round(rand(1,len).*(point_now_y-1).*N_x+1);% -1 , do not use the last line
    X=mod(rind,N_x)+1;
    Y=mod(rind,point_now_y)+1;
    ims=imsf(rind);
    cf = createFit_surf_poly(X, Y, ims,1);
    [m,n]=size(im_buffer);
    [X,Y]=meshgrid(1:m,(1:n)');
    z_cf=feval(cf,X,Y)';
    ind=im_buffer~=-1;
    im_buffer(ind)=im_buffer(ind)-z_cf(ind);
    %     r_adj=mean(mean(z_cf));
end
im_buffer=im_buffer';% for show
% imshow(im_buffer,[])
if exist('show_line','var')
    image_show_buffer=AFM_generate_RGB_image_for_show(im_buffer,point_now_y);
else
    image_show_buffer=AFM_generate_RGB_image_for_show_without_show_line(im_buffer,point_now_y);
end
% line_now=im_buffer(:,point_now_y);


function im_range=im_range_detect(im)
try
    im=im(im~=0);
    N=length(im);
    while(N>500)
        ind=round(1:N/500:N);
        im=im(ind);
        N=length(im);
    end
    zs=sort(im);
    L=length(zs);
    
    ch=round([0.2 0.8].*L);
    minz=zs(ch(1));
    maxz=zs(ch(2));
    im_range=[minz,maxz];
catch
    im_range=[0 1];
end
% if im_range(1)>minz
%     im_range(1)=minz;
% end
% if im_range(2)<maxz
%     im_range(2)=maxz;
% end
function fitresult = createFit_surf_poly(X, Y, ims,N);
[xData, yData, zData] = prepareSurfaceData( X, Y, ims );
ft = fittype( ['poly' num2str(N)  num2str(N) ]);
opts = fitoptions( ft );
opts.Lower = [-Inf -Inf -Inf];
opts.Robust = 'Bisquare';
opts.Upper = [Inf Inf Inf];
fitresult= fit( [xData, yData], zData, ft, opts );
function image_show_buffer=AFM_generate_RGB_image_for_show(im_buffer,line_now)
% image_show_buffer=im_buffer;
% return
% im_buffer=fliplr(im_buffer);
ind=im_buffer~=-1;
im_range=im_range_detect(im_buffer(ind));

im_buffer=flipud(im_buffer);

image_show_buffer=ones(size(im_buffer,2),size(im_buffer,1),3,'uint8').*uint8(128);
%r
tm=(im_buffer-im_range(1))./(im_range(2)-im_range(1)).*255;
tm=flipud(tm);
tm(end,line_now)=0;
image_show_buffer(:,:,1)=uint8(tm');
%g
tm=(im_buffer-im_range(1))./(im_range(2)-im_range(1)).*150;
tm=flipud(tm);
tm(end,line_now)=255;
image_show_buffer(:,:,2)=uint8(tm');
%b
% tm=zeros(size(tm));

tm=(im_buffer-im_range(1))./(im_range(2)-im_range(1)).*0;
tm(end,line_now)=255;
image_show_buffer(:,:,3)=tm';

function image_show_buffer=AFM_generate_RGB_image_for_show_without_show_line(im_buffer,line_now)
% image_show_buffer=im_buffer;
% return
% im_buffer=fliplr(im_buffer);
ind=im_buffer~=-1;
im_range=im_range_detect(im_buffer(ind));

im_buffer=flipud(im_buffer);

image_show_buffer=ones(size(im_buffer,2),size(im_buffer,1),3,'uint8').*uint8(128);
%r
tm=(im_buffer-im_range(1))./(im_range(2)-im_range(1)).*255;
tm=flipud(tm);
% % % tm(end,line_now)=0;
image_show_buffer(:,:,1)=uint8(tm');
%g
tm=(im_buffer-im_range(1))./(im_range(2)-im_range(1)).*150;
tm=flipud(tm);
% % % tm(end,line_now)=255;
image_show_buffer(:,:,2)=uint8(tm');
%b
% tm=zeros(size(tm));

tm=(im_buffer-im_range(1))./(im_range(2)-im_range(1)).*0;
% % % tm(end,line_now)=255;
image_show_buffer(:,:,3)=tm';