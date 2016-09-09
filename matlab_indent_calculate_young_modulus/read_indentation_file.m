function [z_piezo_NM,prc_readout,paras]=read_indentation_file(fn);
dat=importdata(fn);
if length(dat)==1
    data=dat.data;
    
    for k=1:length(dat.textdata)
        str=dat.textdata{k};
        ind=find(str=='%');
        paras{k}=str2num(str(1:ind-1));
        if isempty(paras{k})
            paras{k}=(str(1:ind-1));
        end
    end
    
else
    data=dat;% compatible with old version
    paras=[];
end

nm=data(:,1);
data(nm==0,:)=[];
% data=data(1:end-1,:);

z_piezo_NM=data(:,1);
prc_readout=-data(:,2);
if prc_readout(z_piezo_NM==max(z_piezo_NM))<mean(prc_readout)
    prc_readout=-prc_readout;
end

end
