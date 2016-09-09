clear
close all

E=[
    %brucker	FIB	microsphere	original
156.4304243	74.81	13.25	23.42
3.41E-10	1.44E-05	2.97E-05	3.00E-01
]

H=1:4;

for k=1:4
    semilogx(E(:,k),H([k k]),'-','LineWidth',20)
    hold on
end
% grid on
ylim([0.5 4.5])
xlim([1e-10 1e3])