function ft= createFit_line_poly(x, y,N);
[xData, yData] = prepareCurveData( x, y );

% Set up fittype and options.
ft = fittype( ['poly' num2str(N)  ]);
opts = fitoptions( ft );
opts.Lower = [-Inf -Inf -Inf];
opts.Upper = [Inf Inf Inf];

opts.Robust = 'Bisquare';%'off';%
% Fit model to data.
ft= fit( xData, yData, ft, opts );
% 
% % Plot fit with data.
% figure
% h = plot( fitresult, xData, yData );
% % legend( h, 'y vs. x')
% % Label axes
% xlabel( 'x' );
% ylabel( 'y' );
% grid on
% 
% 
