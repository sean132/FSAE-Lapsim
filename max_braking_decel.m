function [x_table_braking,braking_decel] = max_braking_decel(long_vel_guess,car,x0)
% uses fmincon to minimize the objective function subject to constraints
% optimizes longitudinal acceleration with given lateral acceleration constraint

if nargin == 2 % no initial guess supplied
    %initial guesses
    steer_angle_guess = 0;
    throttle_guess = -1;
    lat_vel_guess = 0;
    yaw_rate_guess = 0;

    kappa_1_guess = -0.02;
    kappa_2_guess = -0.02;
    kappa_3_guess = -0.02;
    kappa_4_guess = -0.02;
    
    x0 = [steer_angle_guess,throttle_guess,long_vel_guess,lat_vel_guess,yaw_rate_guess,kappa_1_guess,...
        kappa_2_guess,kappa_3_guess,kappa_4_guess];
end

% bounds
steer_angle_bounds = [0,0];
throttle_bounds = [-1,-1]; 
long_vel_bounds = [long_vel_guess,long_vel_guess];
lat_vel_bounds = [0,0];
yaw_rate_bounds = [0,0];
kappa_1_bounds = [-0.2,0.2];
kappa_2_bounds = [-0.2,0.2];
kappa_3_bounds = [-0.2,0.2];
kappa_4_bounds = [-0.2,0.2];

A = [];
b = [];
Aeq = [];
beq = [];
lb = [steer_angle_bounds(1),throttle_bounds(1),long_vel_bounds(1),lat_vel_bounds(1),...
    yaw_rate_bounds(1),kappa_1_bounds(1),kappa_2_bounds(1),kappa_3_bounds(1),kappa_4_bounds(1)];
ub = [steer_angle_bounds(2),throttle_bounds(2),long_vel_bounds(2),lat_vel_bounds(2),...
    yaw_rate_bounds(2),kappa_1_bounds(2),kappa_2_bounds(2),kappa_3_bounds(2),kappa_4_bounds(2)];

% scaling
%scaling_factor = [10 1 1 1 1 1 1 1 1];
%scaling_factor = [    8.2535         1   14.0000   0.0998    1.0639   63.0822   57.6075   63.0822   57.6075];
scaling_factor = ones(1,9);
x0 = x0./scaling_factor;
lb = lb./scaling_factor;
ub = ub./scaling_factor;

% objective function: longitudinal acceleration (forwards)
f = @(P) car.long_accel(P,scaling_factor);

% no longitudinal acceleration constraint
constraint = @(P) car.constraint1(P,scaling_factor);

% default algorithm is interior-point
options = optimoptions('fmincon','MaxFunctionEvaluations',2000,'ConstraintTolerance',1e-2,...
    'StepTolerance',1e-10,'Display','notify-detailed');

% fval: objective function value (v^2/r)
% exitflag meaning: 1 = converged, 2 = change in x less than step tolerance
%   (optimality condition not fulfilled, but solution still found
%   0 = function evaluations exceeded (not converging)
%   -2 = no feasible point found 
[x,fval,exitflag,output,lambda,grad,hessian] = fmincon(f,x0,A,b,Aeq,beq,lb,ub,constraint,options);

cond_hessian = cond(hessian)

[engine_rpm,beta,lat_accel,long_accel,yaw_accel,wheel_accel,omega,current_gear,...
Fzvirtual,Fz,alpha,T] = car.equations(x,scaling_factor);

x_braking = [exitflag long_accel lat_accel x omega(1:4) engine_rpm current_gear beta];

x_cell_braking = num2cell(x_braking);
x_table_braking = cell2table(x_cell_braking);
x_table_braking.Properties.VariableNames = {'exitflag','long_accel','lat_accel','steer_angle',...
    'throttle','long_vel','lat_vel','yaw_rate','kappa_1','kappa_2','kappa_3','kappa_4',...
    'omega_1','omega_2','omega_3','omega_4','engine_rpm','current_gear','beta'};

braking_decel = fval/9.81;
end
