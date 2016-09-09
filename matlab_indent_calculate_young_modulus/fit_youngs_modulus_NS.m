function fit_youngs_modulus_NS(xRetrace,yRetrace)
%     NSMU = NSMatlabUtilities();
    %open a force curve file
%     currentFolder = pwd;
%     fileName = strcat(currentFolder,'\Matlab Example Files\hf02-tuna.005');
%     NSMU.Open(fileName);
    %Get force timed plot of channel 1
%     [xData, yData, xLabel, yLabel] = NSMU.CreateForceTimePlot(1, NSMU.METRIC);
    %uncomment below lines if you want to plot the data
    %plot(xData, yData);
    %xlabel(xLabel);
    %ylabel(yLabel);
    
    
%     use
    %Get F vs Z plot of channel 1
%     [xTrace, xRetrace, yTrace, yRetrace, xLabel, yLabel] = NSMU.CreateForceZPlot(1, NSMU.FORCE, 0);
    %uncomment below lines if you want to plot the data
    %plot(xTrace, yTrace);
    %hold on;
    %plot(xRetrace, yRetrace);
    %xlabel(xLabel);
    %ylabel(yLabel);
    
    %get F vs tip-sample seperation plot of channel 1
%     [xTrace, xRetrace, yTrace, yRetrace, xLabel, yLabel] = NSMU.CreateForceZPlot(1, NSMU.FORCE, 1);
    %uncomment below lines if you want to plot the data
    %plot(xTrace, yTrace);
    
%     xRetrace=flipud(xRetrace(:));
%     yRetrace=flipud(yRetrace(:));
    
%     hold on;
figure(100)
    plot(xRetrace, yRetrace);

    hold on;    
    contactPointIndex = FindContactPoint(xRetrace, yRetrace);
    [regionBottom, regionTop] = ComputeContactRegionBounds(xRetrace, yRetrace, contactPointIndex, 10, 70);
    %plot the contact region
    plot(xRetrace,regionTop,'m');
    plot(xRetrace,regionBottom,'m');
    [index1, index2] = ComputeMarkers(xRetrace, yRetrace,regionTop, regionBottom);
    K = ExponentialFit(xRetrace, yRetrace, index1, index2, contactPointIndex);
    PR = 0.34%NSMU.GetPoissonRatio()
    TR = 16%NSMU.GetTipRadius()
    E = GetYoungsModulus(K, PR, TR)
%     NSMU.Close();
    
end


function E = GetYoungsModulus(K, PoissonRatio, TipRadius)
    %Returns YoungsModulus, E in MPa 
    %Use Hertz sphere model to calculate Young's Modulus.
    PoissonRationSqrd = PoissonRatio^2;
    sqrtTipRadius = sqrt(TipRadius);
    %Hertz model: K = 4/3 * E/(1-PR^2) * R ^ 1/2
    E = (1 - PoissonRationSqrd) * .75 * K/sqrtTipRadius;
end
 
function K= ExponentialFit (xData, yData, index1, index2, contactPointIndex)
    K = 0;
    xSize = max(size(xData));
    ySize = max(size(yData));
    if xSize ~= ySize
      error('xData must be the same size as yData.')
    end
    nBins = abs(index2 - index1) + 1;
    startIndex = min(index1,index2);
    endIndex = max(index1,index2);
    %add the contact regions to the vectors
    fitXData = xData(startIndex:endIndex - 1);
    fitYData = yData(startIndex:endIndex - 1);
    %add the contact point to the front of the vectors
    fitXData = [xData(contactPointIndex); fitXData];
    fitYData = [yData(contactPointIndex); fitYData];
    %transfer the data so x=0 is the first data point
    x0 = fitXData(1);
    x1 = fitXData(nBins);
    scale = abs(x1 - x0)/(x1 - x0);
    yMin = fitYData(1);
    %not flipped
    if fitYData(1) <= fitYData(nBins)
        fitXData = (fitXData - x0) * scale;
    %flipped
    else
        yMin = fitYData(nBins);
        fitXData = (x1 - fitXData)*scale;
    end
    %get y in PN
    fitYData = (fitYData - yMin) * 1000;
    %get the first guess, K = mean(F(x)/x^E)
    n = 0;
    for i=1:nBins
        if fitXData(i)>0 && fitYData(i)>=0
            K = K + fitYData(i)/fitXData(i)^1.5;
            n = n+1;
        end
    end
    if n > 0
        K = K /n;
    end
end

function [index1, index2] = ComputeMarkers(xData, yData, regionTop, regionBottom)

ind=find(yData<regionBottom);
index1=ind(end);
ind=find(yData>regionTop);
index2=ind(1);

% 
%     [index1, index2] = deal(-1);
%     nPts = max(size(yData));
%     step = 1;
%     startIndex = 1;
%     endIndex = nPts;
%     %if it's reverse index
%     if xData(1) > xData(nPts)
%         startIndex = nPts;
%         endIndex = 1;
%         step = -1;
%     end
%     i = startIndex;
%     while i~=endIndex
%         if (index1 == -1)
%             if (yData(i) <= regionTop)
%                 index1 = i;
%             end
%         elseif (yData(i) < regionBottom)
%             index2 = i - step;
%             break;
%         end
%         i = i+ step;
%     end
end
function [regionBottom, regionTop] = ComputeContactRegionBounds(xData, yData, contactPointIndex, minForcePercent, maxForcePercent)

    [curveStartIndex, curveEndIndex] = deal(0);
    xSize = max(size(xData));
    ySize = max(size(yData));
   
    if xData(1) > xData(xSize)
        curveStartIndex = xSize;
        curveEndIndex = 1;
    else
        curveStartIndex = 1;
        curveEndIndex = xSize;
    end
    maxRegion = yData(curveEndIndex);
    minRegion = yData(contactPointIndex);
    regionBottom = (maxRegion - minRegion) * minForcePercent / 100 + minRegion;
    regionTop = (maxRegion - minRegion) * maxForcePercent / 100 + minRegion;
end

function [contactPointIndex] = FindContactPoint(xData, yData)
% Finds intersection point of the curve and line connected the first and last point. 
% curveStartIndex, contactPointIndex andcurveEndIndex are 0 if no intersection exists.
%
% Example

    curveStartIndex = 0;
    contactPointIndex = 0;
    curveEndIndex = 0;
    xSize = max(size(xData));
    ySize = max(size(yData));
    %if(xSize == ySize)

    %Method for contact point is subtract the line connected the first and last point from all points in the curve, and use the lowest point
    slope = (yData(ySize) - yData(1))/ (xData(xSize) - xData(1));
    minY = 0;
    for i = 1:ySize
        yVal = yData(i) - slope * xData(i);
        if (i == 1 || yVal < minY )
            minY = yVal;
            contactPointIndex = i;
        end
    end
    %Determine direction of curve. Method assumes end of the contact region is higher that the non-contact region
    if (yData(1)> yData(ySize))
        curveStartIndex = ySize;
        curveEndIndex = 1;
    else
        curveStartIndex = 1;
        curveEndIndex = ySize;
    end;
    %Method is find highest point after contact point
    if curveEndIndex > contactPointIndex
        increment = 1;
    else
        increment = -1;
    end
    maxY = 0;
    i= contactPointIndex + increment
    while i * increment <= curveEndIndex * increment
        if yData(i) > maxY
            maxY = yData(i);
        end
        i = i + increment;
    end
    targetMaxY = (maxY - yData(contactPointIndex)) + yData(contactPointIndex);
    maxIndex = contactPointIndex;
    i= contactPointIndex + increment
    while i * increment <= curveEndIndex * increment
        if (yData(i) > targetMaxY)
            break;
        end
        maxIndex = i;
        i = i + increment;
    end
    curveEndIndex = maxIndex;
end


