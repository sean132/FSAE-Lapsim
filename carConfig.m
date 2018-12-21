function carCell = carConfig()

carParams = struct();
carParams.mass = 177; %170.097; % not including driver
carParams.driver_weight = 72.5748;
carParams.accel_driver_weight = 68.0389;
carParams.wheelbase = 1.524;
carParams.weight_dist = 0.51; % percentage of weight in rear
carParams.track_width = 1.1938;
carParams.wheel_radius = 0.1956; %0.221; %0.232; % should ideally be loaded radius
carParams.cg_height = 0.2794;
carParams.roll_center_height_front = 0.052;
carParams.roll_center_height_rear = 0.0762;
carParams.R_sf = 0.4; %0.3:0.01:0.6; % proportion of roll stiffness in front (not same as LLTD)
carParams.I_zz = 83.28; %kg-m^2

% aero parameters
aeroParams = struct();
aeroParams.cda = 0.787;
aeroParams.cla = 2:.5:3;
aeroParams.distribution = 0.5; % proportion of downforce in front

% engine parameters
eParams = struct();
eParams.redline = 13000; 
eParams.shift_point = 12000; % approximate
% these parameters are non-iterable
eParams.gears = [2.0, 1.63, 1.33, 1.14, 0.95];
eParams.torque_fn = KTM350(); % contains torque curve

% drivetrain parameters
DTparams = struct();
DTparams.final_drive = 9.3016;
DTparams.drivetrain_efficiency = 0.92; % copied from old lapsim
DTparams.G_d1 = 0; % differential torque transfer offset due to internal friction
DTparams.G_d2_overrun = 0; % differential torque transfer gain in overrun (not used now)
DTparams.G_d2_driving = 0; % differential torque transfer gain on power

% brake parameters
Bparams = struct();
Bparams.brake_distribution = 0.7; % proportion of brake torque applied to front
Bparams.max_braking_torque = 800; % total braking torque

% tire parameters
tireParams = struct();
tireParams.gamma = 0; % camber angle
tireParams.p_i = 12; % pressure
% these parameters are non-iterable
load('Fx_combined_parameters_run38_30.mat'); % F_x combined magic formula parameters
tireParams.Fx_parameters = cell2mat(Xbestcell);
load('Fy_combined_parameters_run6_new.mat'); % F_y combined magic formula parameters
tireParams.Fy_parameters = cell2mat(Xbestcell);

% cell array of gridded parameters
[carCell] = parameters_loop(carParams,aeroParams,eParams,DTparams,Bparams,tireParams);