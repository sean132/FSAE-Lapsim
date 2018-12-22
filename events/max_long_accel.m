function [x_accel,long_accel,long_accel_guess] = max_long_accel(long_vel_guess,car,x0)
% uses fmincon to minimize the objective function subject to constraints
% optimizes longitudinal acceleration with given lateral acceleration constraint
% disp('max long accel');
if nargin == 2 % no initial guess supplied
    %initial guesses
    steer_angle_guess = 0;
    throttle_guess = 1;
    lat_vel_guess = 0;
    yaw_rate_guess = 0;

    kappa_1_guess = 0;
    kappa_2_guess = 0;
    kappa_3_guess = 0.01;
    kappa_4_guess = 0.01;
    
    x0 = [steer_angle_guess,throttle_guess,long_vel_guess,lat_vel_guess,yaw_rate_guess,kappa_1_guess,...
        kappa_2_guess,kappa_3_guess,kappa_4_guess];
end

x0(3) = long_vel_guess;

% bounds
steer_angle_bounds = [0,0];
throttle_bounds = [0,1]; 
long_vel_bounds = [long_vel_guess,long_vel_guess];
lat_vel_bounds = [0,0];
yaw_rate_bounds = [0,0];
kappa_1_bounds = [0,0];
kappa_2_bounds = [0,0];
kappa_3_bounds = [0,0.2];
kappa_4_bounds = [0,0.2];

A = [];
b = [];
Aeq = [1 0 0 0 0 0 0 0 0
       0 0 1 0 0 0 0 0 0
       0 0 0 1 0 0 0 0 0
       0 0 0 0 1 0 0 0 0
       0 0 0 0 0 1 0 0 0
       0 0 0 0 0 0 1 0 0];
beq = [0 long_vel_guess 0 0 0 0];
lb = [steer_angle_bounds(1),throttle_bounds(1),long_vel_bounds(1),lat_vel_bounds(1),...
    yaw_rate_bounds(1),kappa_1_bounds(1),kappa_2_bounds(1),kappa_3_bounds(1),kappa_4_bounds(1)];
ub = [steer_angle_bounds(2),throttle_bounds(2),long_vel_bounds(2),lat_vel_bounds(2),...
    yaw_rate_bounds(2),kappa_1_bounds(2),kappa_2_bounds(2),kappa_3_bounds(2),kappa_4_bounds(2)];

% objective function: longitudinal acceleration (forwards)
f = @(P) -car.long_accel(P);

% no lateral acceleration constraint
constraint = @(P) car.constraint1(P);

% default algorithm is interior-point
% options = optimoptions('fmincon','MaxFunctionEvaluations',2000,'ConstraintTolerance',1e-2,...
%     'StepTolerance',1e-10,'Display','notify-detailed');
options = optimoptions('fmincon','MaxFunctionEvaluations',2000,'ConstraintTolerance',1e-2,...
    'StepTolerance',1e-10,'Display','off');

% fval: objective function value (v^2/r)
% exitflag meaning: 1 = converged, 2 = change in x less than step tolerance
%   (optimality condition not fulfilled, but solution still found
%   0 = function evaluations exceeded (not converging)
%   -2 = no feasible point found 
[x,fval,exitflag,output,lambda,grad,hessian] = fmincon(f,x0,A,b,Aeq,beq,lb,ub,constraint,options);
x(9) = x(8);

long_accel_guess = x;

[engine_rpm,beta,lat_accel,long_accel,yaw_accel,wheel_accel,omega,current_gear,...
Fzvirtual,Fz,alpha,T] = car.equations(x);

% generate table of control variable values
x_accel = [exitflag long_accel x(3)*x(5) x omega(1:4) engine_rpm current_gear beta...
    Fz(1:4) alpha(1:4) T(1:4)];
[x_table_accel] = generate_table(x_accel);

long_accel = -fval;