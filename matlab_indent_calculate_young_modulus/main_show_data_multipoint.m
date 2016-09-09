clear
close all

Es=[];
for k=1:10
    filename=['IndentData_PDMS' num2str(k) '.mat']
    load(filename);
    Es=[Es;Esample];
    
    figure(5),
    boxplot(Esample)
    h = findobj(gcf,'tag','Outliers');
    
    xdata = get(h,'XData');
    ydata = get(h,'YData');
    if ~isnan(ydata)
        for n=1:length(ydata)
            ind=Esample==ydata(n);
            Esample(ind)=[];
        end
    end
    Ecl=Esample;
    
    Esc{k}=Ecl;
    
m(k)=mean(Esample)
D(k)=std(Esample)    
end

figure(1)
bar(Es)


figure(2)
bar(m)
hold on
errorbar(m,D)
ylim([0 1.25].*1e-3)
xlim([0.5 10.5])
grid on
