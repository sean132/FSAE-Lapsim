function [x_ss,lat_accel,long_accel,lat_accel_guess] = max_lat_accel(long_vel_guess,car,x0)
% uses fmincon to minimize the objective function subject to constraints
% optimizes lateral acceleration with no longitudinal acceleration constraint
% varies inputs to maximize lateral acceleration
disp('max_lat_accel');
if nargin == 2 % no initial guess supplied
    % initial guesses
    steer_angle_guess = 10; %degrees
    throttle_guess = 0;
    lat_vel_guess = 0.1;
    yaw_rate_guess = 0.5;
    kappa_1_guess = 0;
    kappa_2_guess = 0;
    kappa_3_guess = 0;
    kappa_4_guess = 0;
    
    x0 = [steer_angle_guess,throttle_guess,long_vel_guess,lat_vel_guess,yaw_rate_guess,kappa_1_guess,...
        kappa_2_guess,kappa_3_guess,kappa_4_guess];
end

x0(3) = long_vel_guess;

% bounds
steer_angle_bounds = [0,25];
throttle_bounds = [0,0]; 
long_vel_bounds = [long_vel_guess,long_vel_guess];
lat_vel_bounds = [-3,3];
yaw_rate_bounds = [0,2];
kappa_1_bounds = [0,0];
kappa_2_bounds = [0,0];
kappa_3_bounds = [0,0];
kappa_4_bounds = [0,0];

A = [];
b = [];
Aeq = [0 1 0 0 0 0 0 0 0
       0 0 1 0 0 0 0 0 0
       0 0 0 0 0 1 0 0 0
       0 0 0 0 0 0 1 0 0
       0 0 0 0 0 0 0 1 0
       0 0 0 0 0 0 0 0 1];
beq = [0 long_vel_guess 0 0 0 0];
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

% default algorithm is interior-point
options = optimoptions('fmincon','MaxFunctionEvaluations',1000,'ConstraintTolerance',1e-2,...
    'StepTolerance',1e-10,'Display','notify-detailed');

% objective function: longitudinal velocity times yaw rate (v*v/r = v^2/r)
f = @(P) -P(3)*P(5);                                     

% no longitudinal acceleration constraint
constraint = @(P) car.constraint1(P,scaling_factor);

% fval: objective function value (v^2/r)
% exitflag meaning: 1 = converged, 2 = change in x less than step tolerance
%   (optimality condition not fulfilled, but solution still found
%   0 = function evaluations exceeded (not converging)
%   -2 = no feasible point found 
[x,fval,exitflag,output,lambda,grad,hessian] = fmincon(f,x0,A,b,Aeq,beq,lb,ub,constraint,options);

cond_hessian = cond(hessian);

[engine_rpm,beta,~,long_accel,~,~,~,~,~,...
    omega_1,omega_2,omega_3,omega_4,current_gear,~,~,~,~,...
    Fz_1,Fz_2,Fz_3,Fz_4,alpha_1,alpha_2,alpha_3,alpha_4,T_1,T_2,T_3,T_4] = car.equations(x,scaling_factor);


% unscaling
x = x.*scaling_factor;

lat_accel_guess = x;

% generate table of control variable values
x_ss = [exitflag long_accel x(3)*x(5) x omega_1 omega_2 omega_3 omega_4 engine_rpm current_gear beta...
    Fz_1 Fz_2 Fz_3 Fz_4 alpha_1 alpha_2 alpha_3 alpha_4 T_1 T_2 T_3 T_4];
[x_table_ss] = generate_table(x_ss)

% maximum lateral accel in g's
lat_accel = x(3)*x(5)/9.81;

end

