function [x_table_skid, max_vel_skid, skidpad_time, skid_guess] = max_skidpad_vel(radius,car,x0)
% uses fmincon to minimize the objective function subject to constraints
% optimizes lateral acceleration with no velocity constraint and zero
%   longitudinal acceleration constraint (steady-state skidpad assumption)
disp('max_skidpad_vel');
if nargin == 2 % no initial guess supplied
    % initial guesses 
    steer_angle_guess = 7; % degrees
    throttle_guess = 0.1;
    long_vel_guess = sqrt(9.81*0.3*radius); % approximation method to help convergence
    lat_vel_guess = 0.2;
    yaw_rate_guess = 0.3;

    kappa_1_guess = 0;
    kappa_2_guess = 0;
    kappa_3_guess = 0.01;
    kappa_4_guess = 0.01;

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

% objective function: longitudinal velocity times yaw rate (v*v/r = v^2/r)
f = @(P) -P(3)*P(5);                                     

% longitudinal acceleration constrained to zero
constraint = @(P) car.constraint3(P,radius);

% default algorithm is interior-point
options = optimoptions('fmincon','MaxFunctionEvaluations',5000,'ConstraintTolerance',1e-2,...
    'StepTolerance',1e-10,'Display','notify-detailed');

[x,fval,exitflag] = fmincon(f,x0,A,b,Aeq,beq,lb,ub,constraint,options);

max_vel_skid = x(3);
skid_guess = x;

[engine_rpm,beta,lat_accel,long_accel,yaw_accel,wheel_accel,omega,current_gear,...
Fzvirtual,Fz,alpha,T] = car.equations(x,scaling_factor);

x_skid = [exitflag long_accel x(3)*x(5) x omega(1:4) engine_rpm current_gear beta...
    Fz(1:4) alpha(1:4) T(1:4)];

x_table_skid = generate_table(x_skid);

skidpad_time = (pi*2*radius)/max_vel_skid;