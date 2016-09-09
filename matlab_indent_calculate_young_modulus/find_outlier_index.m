function [ind_b,ind_num]=find_outlier_index(x);
ind_b = abs(x - median(x)) > 3*std(x);
ind_num=find(ind_b==1);
end