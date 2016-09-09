function [z_piezo_NM,prc_readout,paras]=read_indentation_file_brucker(fn,indent_data_length);
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
% data(nm==0,:)=[];
data(isnan(nm),:)=[];
% data=data(1:end-1,:);

z_piezo_NM=[data(:,5); data(:,6)];
prc_readout=[data(:,1); data(:,2)];% the raw data is nm deflection already
% prc_readout=[data(:,3); data(:,4)].*1e-3;% the raw data is force pN deflection already

% z_piezo_NM=[data(:,5);  ];
% prc_readout=[data(:,1);  ];

if prc_readout(z_piezo_NM==max(z_piezo_NM))<mean(prc_readout)
    prc_readout=-prc_readout;
end
%% cut data
if length(z_piezo_NM)>indent_data_length*4
    N=length(z_piezo_NM);
    ind=N/2-indent_data_length*2:N/2+indent_data_length*2;
    ind=round(ind);
    z_piezo_NM=z_piezo_NM(ind);
    prc_readout=prc_readout(ind);
end
end
