clear;



for i = 1:128
    for j = 1:128
        
        temp_data = j;
        a(i,j)=temp_data;
    end
end

y = a(:);

y2 = a';
y2 = y2(:);


img = imread('cameraman.tif');
img = img(1:128,1:128);
img = img(:);

mydata = [y,y2,img];
csvwrite('data.txt', mydata)

