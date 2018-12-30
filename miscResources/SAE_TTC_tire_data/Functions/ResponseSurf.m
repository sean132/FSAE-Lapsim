% Copyright 2017 The MathWorks, Inc.

function [P] = ResponseSurf(Model_parameter,FZ,Ind_parameter,N)
%[P] = ResponseSurf(Model_parameter,FZ,Ind_parameter,N)
%=============================
%This function fits a surface of order N to a given set of data and outputs
%the coefficients in an array P
%=============================
%INPUT ARGUMENTS
%=============================
%Model_parameter = either mu or dimensionless slip stiffness must be
%size(Ind_parameter x Model_parameter) matrix
%FZ = Array of vertical loads (FZ)
%Ind_parameter = independent parameter chosen for fitting, can be
%inclination angle,pressure,rim width etc.
%N = surface order (1,2,3)
%=============================
%OUTPUT ARGUMENTS
%=============================
%P = coefficient vector
%=============================

x = Ind_parameter;
y = FZ;
[x_mat,y_mat] = meshgrid(x,y);
Dim = size(x_mat);
Num = Dim(1)*Dim(2);
x = reshape(x_mat,1,Num);
y = reshape(y_mat,1,Num);
zdata = reshape(Model_parameter,1,Num);
if N==1
    P=fit([x',y'],zdata','poly11','Normalize','off');
elseif N==2
    P=fit([x',y'],zdata','poly22','Normalize','off');
elseif N==3
    P=fit([x',y'],zdata','poly33','Normalize','off');
else
    error('Order of fit must be an integer value from 1 to 3');
end
end

