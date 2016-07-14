function [r, g, b] = AFM_convert_height2RGB(im_height, parameter);
    % created by Ji Ge, 20160706
    % input: im_height, line_now
    % output: r_for_show, g_for_show,b_for_show
    
    %% for test debug
    
    if nargin==0
        %         tic
        load input.mat
        im_height=(imEL)+1000;
        line_now=point_now_y;
        im_height(:,80:128)=-1;
        parameter=[50,1,1,1];
        warning off
    end
    try
        
        %%
        % tic
        im_height = double(im_height);
        line_now=parameter(1);
        show_image=parameter(2);
        fit_order=parameter(3);
        data_save=parameter(4);
        if data_save==1
            save('afm_imshow_input.mat','im_height','parameter');
        end
        % %%
        % [s1,s2]=size(im_height);
        % s=30;
        % ims=imresize(im_height,[s s]);
        % ims=medfilt2(ims,[5 5],'symmetric');
        % x=1:s;
        % y=1:s;
        % [X,Y]=meshgrid(x,y);
        % X=X(:);
        % Y=Y(:);
        % ims=ims(:);
        % cf = createFit_surf_poly(X, Y, ims,1);
        % [m,n]=size(im_height);
        % [X,Y]=meshgrid(1:m,(1:n)');
        % z_cf=feval(cf,X,Y)';
        % ind=im_height~=-1;
        % im_height(ind)=im_height(ind)-z_cf(ind);
        %%
        % ind = im_height ~= -1;
        null_region=im_height == -1;
        
        [s1, s2] = size(im_height);
        nofp=10;% number of points used in curve fitting
        xind_n = round(linspace(1,s1,nofp));
        xind=1:s1;
        for k = 1:s2
            L = im_height(:, k);
            cfL= createFit_line_poly(xind_n', L(xind_n), fit_order);
            L = L-feval(cfL, xind);
            im_height(:, k) = L;
        end
        %%
        %         toc
        % im_height(null_region) = max(im_height(ind));
        %% show image at the grayscale of 0.25:0.75
        n_rangeL=0.15;
        n_rangeH=0.85;
        imd=imresize(im_height,[10,10],'cubic');%nearest
        imd=imd(:);
        rL=min(imd);
        rH=max(imd);
        bin=linspace(rL,rH,512);
        pn=hist(imd,bin);
        ps=cumsum(pn);
        ps=ps./ps(end);
        ind=find(ps>n_rangeL&ps<n_rangeH);
        inds=[ind(1) ind(end)];
        bins=bin(inds);
        cfL=createFit_line_poly_N([n_rangeL n_rangeH] , bins,1);
        height_range_show=feval(cfL,[0,1]);
        %%
        im255 = uint8(normalize_01(im_height,height_range_show(1),height_range_show(2)).*255);
        map = jet(256).*255;
        r = map(im255+1, 1);
        r = reshape(r, size(im255));
        % imshow(r)
        g = map(im255+1, 2);
        g = reshape(g, size(im255));
        b = map(im255+1, 3);
        b = reshape(b, size(im255));
        %%
        r(null_region)=0;
        g(null_region)=0;
        b(null_region)=0;
        %%
        %         toc
        if line_now>=1000
            r(line_now,:)=0;
            g(line_now,:)=0;
            b(line_now,:)=0;
        end
        % return
        
        if show_image==1
            rgb=zeros([size(im255) 3]);
            rgb(:,:,1)=r;
            rgb(:,:,2)=g;
            rgb(:,:,3)=b;
            figure(1)
            imshow(uint8(rgb))
            figure(2)
            imshow(im255)
        end
        
    catch
        r=zeros(size(im_height),'double');
        g=r;
        b=r;
    end
end