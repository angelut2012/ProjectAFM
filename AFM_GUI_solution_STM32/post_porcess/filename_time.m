function st=filename_time();
t=floor(clock);
st=[];
for k=1:length(t)
    st=[st '_' num2str(t(k))];
end
end