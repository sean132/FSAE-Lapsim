
% This script generates a spline describing the racetrack
% It then calculates the curvature as a function of arclength (distance)

%% Map Making Instructions
% insert image of track into solidworks part file
% create style spline in Solidworks (degree 5 seems to work well) on track
% export points by running macro Sourbh-ExtractSplinePoints
% select sketch with spline then click tools->macro->run->select file

% open excel file in Matlab and import X and Y coordinates
% if you didn't click the spline points in order on Solidworks, they may be out of order
% to fix the order, scatter(X,Y) and use the brush to select the first point
% right-click 'Copy Data to Clipboard' and  paste it into an excel file
% continue with the rest of the points in order

% import the re-ordered X and Y coordinates as column vectors
% set degree as one greater than you used for drawing the spline 
% make sure to add nurbs_toolbox to search path
% run this file

%% NURBS Toolbox
%load('bezierpoints.mat');

% spline tools from NURBS Toolbox by D.M. Spink
coefs = [transpose(X); transpose(Y)]; 
degree = 6;
knots = [zeros(1,degree-1) linspace(0,1,numel(X)-degree+2) ones(1,degree-1)];

% creates NURBS (Non-uniform rational basis spline)
nurbs = nrbmak(coefs,knots);

% plotting
subdivisions = 100002;
p = nrbeval(nurbs,linspace(0.0,1.0,subdivisions)); 
plot(p(1,:),p(2,:)); 
hold on
scatter(X,Y);

% evaluating curvature
dY = diff(p(2,:))./diff(linspace(0,1,subdivisions));   % first derivative
ddY = diff(dY)./diff(linspace(0,1,subdivisions-1)); %second derivative
dX = diff(p(1,:))./diff(linspace(0,1,subdivisions)); %first derivative
ddX = diff(dX)./diff(linspace(0,1,subdivisions-1)); %second derivative
dX = dX(1:end-1);
dY = dY(1:end-1);

curvature = (dX.*ddY-ddX.*dY)./(dX.*dX+dY.*dY).^(3/2);

% evaluating arclength
seglength = sqrt(sum(diff(p,[],2).^2,1));
total_arclength = sum(seglength);
arclength = linspace(0,total_arclength,numel(curvature));

% plotting
figure(2)
plot(arclength,curvature);
xlabel('Distance','FontSize',15);
ylabel('Curvature','FontSize',15);

% when satisfied with results, save arclength and curvature data
save('track_autocross_2018.mat','arclength','curvature');
