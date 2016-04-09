clear
close all
load('data_bump_height.mat')

Q(10,:,:)=[];
s=2
bs=1:13;
mtf=Q(bs,s,9);
mtf=abs(mtf)
std_tf=Q(bs,s,6);
bar(mtf,'y')
hold on
errorbar(mtf,std_tf,'m')
set(gca,'XTickLabel',  {'13','23','35','40','41','42','43','101','119','225','226','227','247','268','270'}),
ylabel('error (nm)')
xlabel('bump height (nm)')
return
mch=Q(:,1,7);
std_ch=Q(:,1,8);
m=[mch mtf]
all_std=[std_tf std_ch]
% bar(m)
% hold on
% X=[1:15;1:15;1:15;1:15;1:15;]'
% errorbar(X,m,all_std)
barweb(m,all_std)
% set(gca,'XTickLabel',  {'13','23','35','40','41','42','43','101','119','225','226','227','247','268','270'}),

grid on
legend('AFM','0.3 V',   '0.7 V',  '1.1 V',         '1.5 V', 'Location', 'NorthWest' )
xlabel('bump')
ylabel('height (nm)')
%%%%%%%%%%%%%%%%%
return
d=Q(10:end-1,:,10);
d=abs(d)
figure(10)
mch=mch(10:end-1);
MCH=[mch mch mch mch];
d=d./MCH;
% stem(mch,d)
% grid on
% xlabel('bump height (nm)')
% ylabel('repeatbility (nm, 99%)')

bar(d)
set(gca,'XTickLabel',  {'13','23','35','40','41','42','43','101','119','226','227','247','268'}),%,'270''225',
grid on
legend('0.3 V',         '0.7 V',         '1.1 V',         '1.5 V', 'Location', 'NorthWest' )
xlabel('bump height (nm)')
ylabel('nm')
%%%%%%%%%%%%  voltage
bar(d'.*100)
set(gca,'XTickLabel',  {'0.3',         '0.7',         '1.1',         '1.5'})       ,%'225',
grid on
% legend('0.3 V',         '0.7 V',         '1.1 V',         '1.5 V', 'Location', 'NorthWest' )
xlabel('working voltage (V)')
ylabel('repeatbility (nm)')

ylabel('accuracy (%)')
legend('13','23','35','40','41','42','43','101','119','225','226','227','247','268')

legend('225','226','227','247','268')
% title('accuracy')

% acc=Q(:,:,3);
% reap=Q(:,:,4);
% figure(10)
% bar(acc)
% grid on
% legend('AFM','0.3 V',         '0.7 V',         '1.1 V',         '1.5 V', 'Location', 'NorthWest' )
% xlabel('bump')
% ylabel('nm')
% title('accuracy')
%
% figure(11)
% bar(reap)
% grid on
% legend('AFM','0.3 V',         '0.7 V',         '1.1 V',         '1.5 V', 'Location', 'NorthWest' )
% xlabel('bump')
% ylabel('nm')
% title('repeatbility')

