function position=AFM_scan_set_ROI(im_buffer,position);
try
XL=position(1);
YL=position(2);
DX=position(3);
DY=position(4);
[N_x,N_y]=size(im_buffer);
[image_show_buffer,im_buffer]=AFM_dip_for_show(im_buffer,1,N_y);
imwrite(image_show_buffer,'SetScanROI.bmp')
if length(position)>4
    figure(2)
    clf
    imshow(image_show_buffer,[])
    
    h_rect=imrect;%(g_handles.axes_image);
    % P=wait(h);
    P=h_rect.getPosition();
    % h.delete()
    % x0,y0,DX,DY
    % clf(g_handles.axes_image)
    P([1 3])=P([1 3])./N_x.*DX;
    P([2 4])=P([2 4])./N_y.*DY;
    
    XL=XL+P(1);
    YL=YL+P(2);
    DX=P(3);
    DY=P(4);
    position=[XL,YL,DX,DY]';
end
catch ME
    %fprintf('error') ;
    out=[num2str(ME.stack(1).line) ': ' ME.message];
    position=double(out);
end

end