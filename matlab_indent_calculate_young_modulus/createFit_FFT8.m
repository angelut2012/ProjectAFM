function [fitresult, gof] = createFit_FFT8(n, ny)
[xData, yData] = prepareCurveData( n, ny );

ft = fittype( 'fourier8' );%
opts = fitoptions( ft );
opts.Display = 'Off';
opts.Lower = [-Inf -Inf -Inf -Inf -Inf -Inf -Inf -Inf -Inf -Inf -Inf -Inf -Inf -Inf -Inf -Inf -Inf -Inf];
opts.StartPoint = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0.00154910880354526];
opts.Upper = [Inf Inf Inf Inf Inf Inf Inf Inf Inf Inf Inf Inf Inf Inf Inf Inf Inf Inf];

% Fit model to data.
[fitresult, gof] = fit( xData, yData, ft, opts );

% % Plot fit with data.
% figure( 'Name', 'untitled fit 1' );
% h = plot( fitresult, xData, yData );
% legend( h, 'ny vs. n', 'untitled fit 1', 'Location', 'NorthEast' );
% % Label axes
% xlabel( 'n' );
% ylabel( 'ny' );
% grid on


