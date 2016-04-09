function out=AFM_dip_show_image(imHL,imHR,imEL,imER,point_now_x,point_now_y,str);
warning off
out=' ';
try
    eval(char(str));
    % save('input.mat','imHL','imHR','imEL','imER','point_now_x','point_now_y')
    % load input.mat
    F={imHL',imHR',imEL',imER'};
    
    point_now_x=double(point_now_x);
    point_now_y=double(point_now_y);
    % F={imHL,imHR,imEL,imER};
    figure(1)
    clf
    
    for k=1:length(F)
        %     f=F{k};
        %     im_buffer=load(f);
        im_buffer=F{k};
        im_buffer=double(im_buffer);
        %     im_buffer=im_buffer';
        
        %swap xy
        %     t=point_now_x;
        %     point_now_x=point_now_y;
        %     point_now_y=t;
        
        [image_show_buffer,im_buffer_out]=AFM_dip_for_show(im_buffer,point_now_x,point_now_y,'show_line');
%          [image_show_buffer,im_buffer_out]=AFM_dip_for_show(im_buffer,point_now_x,point_now_y);
%         im=im_buffer_out;
%         [N_x,N_y]=size(im);
%         im_buffer_out=medfilt2(im_buffer_out,[5 5]);
        
        
        K=[321
            323
            322
            324];
        subplot(K(k))
%         image_show_buffer(:,:,1)=image_show_buffer(:,:,1)';
%         image_show_buffer(:,:,2)=image_show_buffer(:,:,2)';
%         image_show_buffer(:,:,3)=image_show_buffer(:,:,3)';
        imshow(image_show_buffer,[])
        hold on
        plot(point_now_x, point_now_y,'r*','MarkerSize',10)
        KL=[325
            325
            326
            326];
        
%         point_now_y=100
%         load('er.mat')
% load('HL.mat')
% ims=im_buffer_out+er;
%         im_buffer=ims;
        line_now_show=im_buffer(abs(point_now_y),1:end-1);
        line_now_show=AFM_line_for_show(line_now_show,1);
        line_now_show=line_now_show-line_now_show(1);
        c='bgbg';
        subplot(KL(k))
        plot(1:length(line_now_show),line_now_show,[c(k) '.-'])
        hold on
        xlim([1 length(line_now_show)])
        grid on
    end
%     try
catch ME
    %fprintf('error') ;
    out=[num2str(ME.stack(1).line) ': ' ME.message]
end



function    line_now_show=AFM_line_for_show(L,order);
x=1:length(L);
cfL=createFit_line_poly(x,L,order);
v_cfL=feval(cfL,x);
v_cfL=v_cfL';

sL=sort(L);
N=length(sL);
ch=round([0.3 0.7].*N);
ch=round(ch);
mL=mean(sL(ch(1):ch(2)));
line_now_show=L-v_cfL+mL;
% end
% figure(2)
% clf
% surf(im_buffer)

%, 'Parent', g_handles.axes_image); % hAxes = finobj(gcf, 'type' ,'axes');
% imshow(image_show_buffer_E,[], 'Parent', g_handles.axes_image_E); % hAxes = finobj(gcf, 'type' ,'axes');
% % im_buffer
%             imagesc(im_buffer', 'Parent', g_handles.axes_image); % hAxes = finobj(gcf, 'type' ,'axes');
%             imagesc(im_buffer_E','Parent', g_handles.axes_image_E); % hAxes = finobj(gcf, 'type' ,'axes');
%