clear
out=[]
for k=1
    
    run(['bn' num2str(k) '.m'])
out=[out;para_evaluation(Vtr,Vtf)];
end

out