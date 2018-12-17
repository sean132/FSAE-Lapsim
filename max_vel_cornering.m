function [x_corner_vel,max_vel_corner,vel_corner_guess] = max_vel_cornering(radius,max_vel,car,x0)
% uses fmincon to minimize the objective function subject to constraints
% optimizes lateral acceleration with no velocity constraint and no
%   longitudinal acceleration constraint (in contrast to skidpad)

if nargin == 3 % no initial guess supplied
    % initial guesses
    steer_angle_guess = 5; % degrees
    throttle_guess = 0;
    long_vel_guess = sqrt(9.81*0.3*radius); % approximation method to help convergence
    lat_vel_guess = 0.2; 
    yaw_rate_guess = 0.3;

    kappa_1_guess = 0;
    kappa_2_guess = 0;
    kappa_3_guess = 0;
    kappa_4_guess = 0;
    
    x0 = [steer_angle_guess,throttle_guess,long_vel_guess,lat_vel_guess,yaw_rate_guess,kappa_1_guess,...
        kappa_2_guess,kappa_3_guess,kappa_4_guess];
end

% bounds
steer_angle_bounds = [0,25];
throttle_bounds = [0,0]; 
long_vel_bounds = [0,max_vel];
lat_vel_bounds = [-3,3];
yaw_rate_bounds = [0,2];
kappa_1_bounds = [0,0];
kappa_2_bounds = [0,0];
kappa_3_bounds = [0,0];
kappa_4_bounds = [0,0];

A = [];
b = [];
Aeq = [0 1 0 0 0 0 0 0 0
       0 0 0 0 0 1 0 0 0
       0 0 0 0 0 0 1 0 0
       0 0 0 0 0 0 0 1 0
       0 0 0 0 0 0 0 0 1];
beq = [0 0 0 0 0];
lb = [steer_angle_bounds(1),throttle_bounds(1),long_vel_bounds(1),lat_vel_bounds(1),...
    yaw_rate_bounds(1),kappa_1_bounds(1),kappa_2_bounds(1),kappa_3_bounds(1),kappa_4_bounds(1)];
ub = [steer_angle_bounds(2),throttle_bounds(2),long_vel_bounds(2),lat_vel_bounds(2),...
    yaw_rate_bounds(2),kappa_1_bounds(2),kappa_2_bounds(2),kappa_3_bounds(2),kappa_4_bounds(2)];

% scaling
scaling_factor = ones(1,9);
x0 = x0./scaling_factor;
lb = lb./scaling_factor;
ub = ub./scaling_factor;

% default algorithm is interior-point
options = optimoptions('fmincon','MaxFunctionEvaluations',5000,'ConstraintTolerance',1e-2,...
    'StepTolerance',1e-10,'Display','notify-detailed');

% objective function: longitudinal velocity times yaw rate (v*v/r = v^2/r)
f = @(P) -P(3)*P(5);                                     

% no longitidunal acceleration constraint
constraint = @(P) car.constraint5(P,radius,scaling_factor);

% fval: objective function value (v^2/r)
% exitflag meaning: 1 = converged, 2 = change in x less than step tolerance
%   (optimality condition not fulfilled, but solution still found
%   0 = function evaluations exceeded (not converging)
%   -2 = no feasible point found 

[x,fval,exitflag] = fmincon(f,x0,A,b,Aeq,beq,lb,ub,constraint,options);
vel_corner_guess = x;

[engine_rpm,beta,~,long_accel,~,~,~,~,~,...
    omega_1,omega_2,omega_3,omega_4,current_gear,~,~,~,~,...
    Fz_1,Fz_2,Fz_3,Fz_4,alpha_1,alpha_2,alpha_3,alpha_4,T_1,T_2,T_3,T_4] = car.equations(x,scaling_factor);

% unscaling
x = x.*scaling_factor;
max_vel_corner = x(3);

x_corner_vel = [exitflag long_accel x(3)*x(5) x omega_1 omega_2 omega_3 omega_4 engine_rpm current_gear beta...
    Fz_1 Fz_2 Fz_3 Fz_4 alpha_1 alpha_2 alpha_3 alpha_4 T_1 T_2 T_3 T_4];
[x_table_corner_vel] = generate_table(x_corner_vel);

