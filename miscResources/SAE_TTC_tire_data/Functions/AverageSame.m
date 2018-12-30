% Copyright 2017 The MathWorks, Inc.

function [A] = AverageSame(A,B)
%[A] = AverageSame(A,B)S
%======================
% This function takes vectors A and B as inputs and identifies indices of A
% where B values coincide. An average is then taken of these values and
% placed back into the vector at each of the indices where absolute values
% of B coincide. The sign is then changed to be the same as B.
% The main application of this is removing asymmetric effects from tyre
% data for the Non-Dimensional Tyre model
%======================
%INPUT ARGUMENTS
%======================
%A = input vector of values to be averaged
%B = reference vector where same-magnitude values are identified. B remains
%    unchanged.
%======================
%OUTPUT ARGUMENTS
%======================
%A = output vector containing the averaged values of A
%=========================
Babs = abs(B);
Aabs = abs(A);
for Bidx = 1:length(Babs)
    BiEqB_Idx = ismember(Babs, Babs(Bidx));
    A(BiEqB_Idx) = mean(Aabs(BiEqB_Idx));
end
A = A.*sign(B);
end
