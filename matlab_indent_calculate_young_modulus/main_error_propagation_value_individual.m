clear
Delta_k=8.65
R=53.4730 
k=44.72
dtip=500/k
dsample=10
Z=dtip+dsample

DK=(3*Delta_k*dtip)/(4*R^(1/2)*(Z - dtip)^(3/2))

Delta_Z=0.5
DZ =-(9*Delta_Z*dtip*k)/(8*R^(1/2)*(Z - dtip)^(5/2))
Delta_R=7.4
DR =-(3*Delta_R*dtip*k)/(8*R^(3/2)*(Z - dtip)^(3/2))
sqrt(sum([DK DR DZ].^2))