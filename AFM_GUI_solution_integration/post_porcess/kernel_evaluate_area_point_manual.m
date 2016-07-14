

sIM_flat=IM_flat;
P=[];    

for k=1:length(IM_flat)
    k
    
%     [dy(k),dx(k),dp(k)]= dftregistration(IM_flat{1},IM_flat{k},0.5);
    
%     dx=round(dx);
%     dy=round(dy);
    if ~exist(['PO_' fn num2str(B) '.mat'],'file')

%         sIM_flat{k}(1:5,:)=nan,
        fh=figure(4);
        clf
%         imshow(sIM_flat{k},[])
gth=50
sIM_flat{k}(sIM_flat{k}>gth)=gth;
        imagesc(sIM_flat{k})
        title(num2str(k))
        set(fh,'Position',[  1   1   814   713])
        h=imrect;
        wait(h);
        if k==1 
            saveas(fh,['bump_' fn num2str(B)  '.tif'])
        end
  PO{k}=h.getPosition();      
%   h.setPosition(PO{k})
    else
        load(['PO_' fn num2str(B) '.mat'])
    end



% PO=[      219   199    14    26];
PO{k}=round(PO{k});
 sx=PO{k}(1):PO{k}(1)+PO{k}(3);
sy=PO{k}(2):PO{k}(2)+PO{k}(4);



sx=round(sx);
sy=round(sy);    
sx=range_limit(sx,1,size(IM_flat{k},2));
sy=range_limit(sy,1,size(IM_flat{k},1));
    
    temp=IM_flat{k}(sy,sx);
    fw=[1 2 1
        2 4 2
        1 2 1];
    fw=fw./sum(fw(:));
    temp=filter2(fw,temp);
    v=max(temp(:));%-min(temp(:));
%     tem{k}=IM_flat{k}(sy+dy(k),sx+dx(k));
    P=[P v];
    surf(temp)
    
%     P=[P; temp(:)];
    
%     IM_flat_re{k}=imfftreconstruct(IM_flat{k},-dy(k),-dx(k),-dp(k));
%     imshow(IM_flat_re{k},[])
    % pause(0.5)
end
save(['PO_' fn num2str(B) '.mat'],'PO','P')
% for k=s
%      P=[tem{k}(:)];
% P(P==0)=[];
P=P'
figure(10)
plot(P,'*-')
figure(11)
imshow(IM_flat{1}(6:end,:),[])
surf(IM_flat{1}(6:end,:))
colormap default
return
% P([1 8])=[]
hs=P;
M=mean(hs);
d=60;
hs(abs(hs-M)>d)=[];

STD=std(hs);
err_P=max(P)-mean(P);
err_N=min(P)-mean(P);
% figure
[pc,bin]=hist(hs,min(hs):2:max(hs));
% cftool(bin,pc)
ft=createFit_gaussian(bin, pc);
% n=1.959963984540
n=2.575829303549;
ft.b1
ft.c1*n
% b(k)=ft.b1 ;
% c(k)=ft.c1*n;
% end
% b(b==0)=[];
% c(c==0)=[];
return
%  H=P;


%