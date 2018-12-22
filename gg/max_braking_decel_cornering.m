function [x_braking,long_decel,braking_decel_guess] = max_braking_decel_cornering(long_vel_guess,lat_accel_value,car,x0)
% uses fmincon to minimize the objective function subject to constraints
% optimizes longitudinal acceleration with given lateral acceleration constraint

if nargin == 3 % no initial guess supplied
    % initial guesses
    steer_angle_guess = 0.01;
    throttle_guess = -0.1;
    lat_vel_guess = -0.1;
    yaw_rate_guess = 0.01;

    kappa_1_guess = -0.01;
    kappa_2_guess = -0.01;
    kappa_3_guess = -0.01;
    kappa_4_guess = -0.01;
    
    x0 = [steer_angle_guess,throttle_guess,long_vel_guess,lat_vel_guess,yaw_rate_guess,kappa_1_guess,...
        kappa_2_guess,kappa_3_guess,kappa_4_guess];
end

x0(3) = long_vel_guess;

% bounds
steer_angle_bounds = [0,25];
throttle_bounds = [-1,0];
long_vel_bounds = [long_vel_guess-0.1,long_vel_guess+0.1];
lat_vel_bounds = [-3,3];
yaw_rate_bounds = [0,2];
kappa_1_bounds = [-0.2,0];
kappa_2_bounds = [-0.2,0];
kappa_3_bounds = [-0.2,0];
kappa_4_bounds = [-0.2,0];

A = [];
b = [];
Aeq = [0 0 1 0 0 0 0 0 0];
beq = [long_vel_guess];
lb = [steer_angle_bounds(1),throttle_bounds(1),long_vel_bounds(1),lat_vel_bounds(1),...
    yaw_rate_bounds(1),kappa_1_bounds(1),kappa_2_bounds(1),kappa_3_bounds(1),kappa_4_bounds(1)];
ub = [steer_angle_bounds(2),throttle_bounds(2),long_vel_bounds(2),lat_vel_bounds(2),...
    yaw_rate_bounds(2),kappa_1_bounds(2),kappa_2_bounds(2),kappa_3_bounds(2),kappa_4_bounds(2)];

% objective function: longitudinal acceleration (forwards)
f = @(P) car.long_accel(P);
% constrained to lateral acceleration value
constraint = @(P) car.constraint4(P,lat_accel_value);
opts = setOptimoptions(1000);
% fval: objective function value (v^2/r)
[x,fval,exitflag,output,lambda,grad,hessian] = fmincon(f,x0,A,b,Aeq,beq,lb,ub,constraint,opts);

[engine_rpm,beta,lat_accel,long_accel,yaw_accel,wheel_accel,omega,current_gear,...
Fzvirtual,Fz,alpha,T] = car.equations(x);

braking_decel_guess = x;

% generate vector of control variable values
x_braking = [exitflag long_accel lat_accel x omega(1:4) engine_rpm current_gear beta...
    Fz(1:4) alpha(1:4) T(1:4)];

long_decel = long_accel/9.81;

end

