function [out_str,out_data]=StringEval(in_str,in_data);
% warning off
out_str=' ';
out_data=in_data;
try
    out_str=evalc(char(in_str));
catch ME
    %fprintf('error') ;
    out_str=[num2str(ME.stack(1).line) ': ' ME.message];
end