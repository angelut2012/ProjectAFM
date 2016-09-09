function [z_piezo_NM,prc_readout]=read_indentation_file_Asylum(d_all,para,n)
amp=1e9;
pfn=['d_all.Image0000'];
str=num2str(n);
pfn((end-length(str))+1:end)=str;
%
%     fn_store{n}=pfn;
    para.pfn=pfn;
    %% read data
%     [z_piezo_NM,prc_readout,paras]=read_indentation_file(pfn);
   
z_piezo_NM_E=eval([pfn 'ZSnsr_Ext']);
prc_readout_E=eval([pfn 'DeflV_Ext']);

z_piezo_NM_R=eval([pfn 'ZSnsr_Ret']);
prc_readout_R=eval([pfn 'DeflV_Ret']);

z_piezo_NM=[z_piezo_NM_E;z_piezo_NM_R];
prc_readout=[prc_readout_E;prc_readout_R];
    z_piezo_NM=clear_nan(z_piezo_NM);
      prc_readout=clear_nan(prc_readout);
    
    z_piezo_NM=z_piezo_NM.*amp;% convert to nm
    
%     prc_readout=prc_readout;% V.*amp
    
end
    