clear
close
f='PRC_indent_test.007.txt.xls'
f='d7.xlsx'
d=importdata(f);
d=d.data;
da=d;%;.data;
x=da(:,5);
y=da(:,1);

x=x-x(end);
y=y-y(end);
plot(x,y,'*-')