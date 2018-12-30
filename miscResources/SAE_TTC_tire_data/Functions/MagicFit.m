% Copyright 2017 The MathWorks, Inc.

function [B,C,D,E,resnorm] = MagicFit(F,Slip)
%[B,C,D,E,resnorm] = MagicFit(F,Slip)
%=========================
%MAGIC FIT
%=========================
%This function takes in raw non-dimensionalised slip and force data and
%then 'fits' coefficients B and E for the magic formula to the data set.
%This is done by re-casting it as an optimization problem and using
%fminsearch to drive error between the fit and raw data to zero.
%===============
% INPUT ARGUMENTS
%===============
%F = concatenated non-dimensionalised force vector.
%Slip = concantenated non-dimensionalised slip vector.
%* Note that the Force and Slip quantities must correspond in the
%  concatenated vectors
%===============
%OUTPUT ARGUMENTS
%===============
%B = B coefficient in the magic formula
%C = C coefficient in the magic formula
%D = D coefficient in the magic formula
%E = E coefficient in the magic formula

%Val = minimisation value.
%===============

%lsqcurvefit fits the Magic Formula to our datapoints
opts = optimset('Display','off');
[Coeff,resnorm,residual] = lsqcurvefit(@MagicFormula,[0.55 1.1],Slip,F,[],[],opts);

B = Coeff(1);
E = Coeff(2);

%Generating the Magic Formula Parameters C and D from Constraints: 
C=1/B;
D=1;

%% use me for debugging
% figure
% plot(Slip,F,'.');
% hold on
% plot(Slip,MagicFormula(Coeff,Slip));

end

