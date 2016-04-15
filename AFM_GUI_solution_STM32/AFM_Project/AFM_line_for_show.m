function    line_show=AFM_line_for_show(line_in,fit_order,index_base_point);
x=1:length(line_in);
cfL=createFit_line_poly(x,line_in,fit_order);
v_cfL=feval(cfL,x);
v_cfL=v_cfL';

sL=sort(line_in);
N=length(sL);
ch=round([0.3 0.7].*N);
ch=round(ch);
mL=mean(sL(ch(1):ch(2)));
line_show=line_in-v_cfL+mL;
if (nargin==3)
    line_show=line_show-line_in(index_base_point);
end
end