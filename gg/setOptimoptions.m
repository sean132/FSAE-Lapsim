function opts = setOptimoptions(numIter)
% opts = optimoptions('fmincon','MaxFunctionEvaluations',1000,'ConstraintTolerance',1e-2,...
%     'StepTolerance',1e-10,'Display','notify-detailed');

% default algorithm is interior-point
% options = optimoptions('fmincon','MaxFunctionEvaluations',1000,'ConstraintTolerance',1e-2,...
%     'StepTolerance',1e-10,'Display','notify-detailed');
opts = optimoptions('fmincon','MaxFunctionEvaluations',numIter,'ConstraintTolerance',1e-2,...
    'StepTolerance',1e-10,'Display','off');


% exitflag meaning: 1 = converged, 2 = change in x less than step tolerance
%   (optimality condition not fulfilled, but solution still found
%   0 = function evaluations exceeded (not converging)
%   -2 = no feasible point found 