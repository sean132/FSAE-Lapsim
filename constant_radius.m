function [x_table, x_guess] = constant_radius(radius,long_vel_value,car,x0)
% uses fmincon to minimize the objective function subject to constraints
% optimizes lateral acceleration with no velocity constraint and zero
%   longitudinal acceleration constraint (steady-state skidpad assumption)

if nargin == 3 % no initial guess supplied
    % initial guesses
    steer_angle_guess = 3; % degrees
    throttle_guess = .2;
    long_vel_guess = long_vel_value; 
    lat_vel_guess = 0.1;
    yaw_rate_guess = long_vel_value/radius;

    kappa_1_guess = 0;
    kappa_2_guess = 0;
    kappa_3_guess = 0.001;
    kappa_4_guess = 0.001;

    x0 = [steer_angle_guess,throttle_guess,long_vel_guess,lat_vel_guess,yaw_rate_guess,kappa_1_guess,...
        kappa_2_guess,kappa_3_guess,kappa_4_guess];
end

% bounds
steer_angle_bounds = [0,25];
throttle_bounds = [0,1]; 
long_vel_bounds = [0,25];
lat_vel_bounds = [-3,3];
yaw_rate_bounds = [0,2];
kappa_1_bounds = [0,0];
kappa_2_bounds = [0,0];
kappa_3_bounds = [0,0.5];
kappa_4_bounds = [0,0.5];

A = [];
b = [];
Aeq = [0 0 0 0 0 1 0 0 0
       0 0 0 0 0 0 1 0 0];
beq = [0 0];
lb = [steer_angle_bounds(1),throttle_bounds(1),long_vel_bounds(1),lat_vel_bounds(1),...
    yaw_rate_bounds(1),kappa_1_bounds(1),kappa_2_bounds(1),kappa_3_bounds(1),kappa_4_bounds(1)];
ub = [steer_angle_bounds(2),throttle_bounds(2),long_vel_bounds(2),lat_vel_bounds(2),...
    yaw_rate_bounds(2),kappa_1_bounds(2),kappa_2_bounds(2),kappa_3_bounds(2),kappa_4_bounds(2)];

% scaling
scaling_factor = ones(1,9);
x0 = x0./scaling_factor;
lb = lb./scaling_factor;
ub = ub./scaling_factor;

% objective function: longitudinal velocity times yaw rate (v*v/r = v^2/r)
f = @(P) -P(3)*P(5);                                     

% longitudinal acceleration constrained to zero, velocity and yaw rate
% constrained for given radius
constraint = @(P) car.constraint8(P,radius,long_vel_value,scaling_factor);

% default algorithm is interior-point
options = optimoptions('fmincon','MaxFunctionEvaluations',5000,'ConstraintTolerance',1e-2,...
    'StepTolerance',1e-10,'Display','notify-detailed');

[x,fval,exitflag] = fmincon(f,x0,A,b,Aeq,beq,lb,ub,constraint,options);

% unscaling
x = x.*scaling_factor;
max_vel_skid = x(3);
x_guess = x;

[engine_rpm,beta,~,long_accel,~,~,~,~,~,...
    omega_1,omega_2,omega_3,omega_4,current_gear,~,~,~,~,...
    Fz_1,Fz_2,Fz_3,Fz_4,alpha_1,alpha_2,alpha_3,alpha_4,T_1,T_2,T_3,T_4] = car.equations(x,scaling_factor);

x_skid = [exitflag long_accel x(3)*x(5) x omega_1 omega_2 omega_3 omega_4 engine_rpm current_gear beta...
    Fz_1 Fz_2 Fz_3 Fz_4 alpha_1 alpha_2 alpha_3 alpha_4 T_1 T_2 T_3 T_4];

x_table = generate_table(x_skid);
