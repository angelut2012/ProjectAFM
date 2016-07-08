function out=convert_temperature_adc2degree(T);
VCC = 5.0;
BIT18MAX=2^18;
Sensitivity_VperC = 0.02;
T = T ./ BIT18MAX.* VCC;

out = (T - 0.5) / Sensitivity_VperC + 25;% - 2.4;
end

