% Copyright 2017 The MathWorks, Inc.

function [F] = MagicFormula(Coeff,Slip)
%[F] = MagicFormula(Coeff,Slip)
%========================
%MAGIC FORMULA
%========================
%Magic formula function as per constraints defined in Patton (2013)
%=======================
%INPUT ARGUMENTS
%=======================
% Coeff = a 1x2 vector containing coefficients B and E
% Slip = Slip quantity (non-dimensional)
% constraints: C=1/B; D=1; You can adjust this Formula if necessary 
%=======================
%OUTPUT ARGUMENTS
%=======================
%F = non-dimensionalised force
%=======================
B = Coeff(1);
E = Coeff(2);
F = sin(1/B*atan(B*(1-E)*Slip+E*atan(B.*Slip)));
end

