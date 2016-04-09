clear
% load('..\bin\input.mat')
% load('input.mat')
% imHL=double(imHL);
% imHL=load('AFM_HL.txt');
% imHL=imHL(:,1:120);

% load input_20150504022957.mat
load input_20150504025252.mat
load input_20150504032003.mat
% load input_20150504050449.mat
load input_20150504063740.mat
% point_now_x=60;
% point_now_y=10;
imHL=imHL+imEL.*0.2;
surf(imHL)
tic
AFM_dip_show_image(imHL,imHR,imEL,imER,point_now_x,point_now_y,' ');
toc
in_str='out_data=load(''AFM_parameter.txt\'');'
[out_str,out_data]=StringEval(in_str,25)



%        try
%           fid = fopen(file);
%           fread(fid);
%        catch ME
% %           fclose(fid);
%           x=ME.message;
% %           rethrow(ME)
%        end

% 0: 341 231
% x150: 340 455
% Y: 116 222
% xy150 113 449
sqrt(1117^2+22^2)/130
p= 8.275678746215467%pixel/um
x=(455-231)/p
y=sqrt((341-116)^2+(231-222)^2)/p

xy=sqrt((341-113)^2+(231-449)^2)/p
38.117584814855825
y*sqrt(2)
38.480530479606308


z=[467 200
    290 200]
z=(467-290)/p

27.067266247186911
27.209844045785250
21.387973775678940