function x=clear_nan(x);
x(isnan(x))=[];
end