clear
mu = 1;     % Population mean
sigma = 2;  % Population standard deviation
n = 10000;    % Sample size

rng default   % For reproducibility
x = normrnd(mu,sigma,n,1);  % Random sample from population

xbar = mean(x)  % Sample mean
s = std(x)      % Sample standard deviation
t = (xbar - mu)/(s/sqrt(n))
n-1
p = 1-tcdf(t,n-1)
% [h,ptest] = ttest(x,mu,0.05,'right')