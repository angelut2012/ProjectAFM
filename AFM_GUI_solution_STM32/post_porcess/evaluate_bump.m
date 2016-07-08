clear
out=[]
N=15
% Q=zeros(16,4,10);
ind= [   12    10    15     9    11     5     8     4     3     2     7     6    13    14     1   ]%1:N1:N
ind=[]
for kk=2:N
    k=kk;%ind(kk);
    %
    
    run(['bn' num2str(k) '.m'])
    c=[ 0    6    11    16    21]
    Vdf=[0.3:0.4:1.5]
    H(k)=mean(Vtr);
    p={};
    for m=1:length(c)-1
        st=c(m)+1;
        se=c(m+1);
        par=para_evaluation(Vtr,Vtf(st:se));
        P=[Vdf(m) par];
        Q(kk,m,:)=P;
        out=[out; P];
        
    end
    
    out=[out;zeros(size(P))];
end
[H, ind]=sort(H)
% %
% % PP={};
% % for k=1:N
% %         for m=1:length(c)-1
% %              PP{(k-1)*N}=P{ind(k)};
% %              end
% % end
% % out=[];
% % for k=1:N
% %             for m=1:length(c)-1
% %                 PP{k}{m}
% %       out=[out;PP{k}{m}];
% %             end
% %        out=[out;zeros(1,10)];
% %     end
% % %     out=[out;zeros(size(par))];
% %




% for k=1:16
%
%     run(['bn' num2str(k) '.m'])
% out=[out;para_evaluation(Vtr,Vtf)];
% end
