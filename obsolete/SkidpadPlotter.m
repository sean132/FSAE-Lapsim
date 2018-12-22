%% Parameters
clearvars;clc;
% input car parameters here
% input parameters you want to iterate through as vectors

% general parameters (all in metric: kg, m, etc.)
mass = 170.097; % not including driver
driver_weight = 72.5748;
accel_driver_weight = 68.0389;
wheelbase = 1.524;
weight_dist = 0.48; % percentage of weight in rear
track_width = 1.1938;
wheel_radius = 0.221; %0.232; % should ideally be loaded radius
cg_height = 0.3048;
roll_center_height_front = 0.052;
roll_center_height_rear = 0.0762;
R_sf = 0.5; %0.3:0.01:0.6; % proportion of roll stiffness in front (not same as LLTD)
I_zz = 83.28; %kg-m^2

% aero parameters
cda = 0.9;
cla = 1.771;
distribution = 0.6; % proportion of downforce in front

% engine parameters
redline = 13000; 
shift_point = 12000; % approximate
% these parameters are non-iterable
gears = [2.0, 1.63, 1.33, 1.14, 0.95];
torque_fn = KTM350(); % contains torque curve

% drivetrain parameters
final_drive = 9.3016;
drivetrain_efficiency = 0.92; % copied from old lapsim
G_d1 = 0; % differential torque transfer offset due to internal friction
G_d2_overrun = 0; % differential torque transfer gain in overrun (not used now)
G_d2_driving = 0; % differential torque transfer gain on power

% brake parameters
brake_distribution = 0.7; % proportion of brake torque applied to front
max_braking_torque = 800; % total braking torque

% tire parameters
gamma = 0; % camber angle
p_i = 12; % pressure
% these parameters are non-iterable
load('Fx_combined_parameters_run38_30.mat'); % F_x combined magic formula parameters
Fx_parameters = cell2mat(Xbestcell);
load('Fy_combined_parameters_run38_30.mat'); % F_y combined magic formula parameters
Fy_parameters = cell2mat(Xbestcell);

[car_cell] = parameters_loop(mass,driver_weight,accel_driver_weight,wheelbase,weight_dist,track_width,...
    wheel_radius,cg_height,roll_center_height_front,roll_center_height_rear,R_sf,I_zz,cda,cla,...
    distribution,redline,shift_point,gears,torque_fn,final_drive,drivetrain_efficiency,G_d1,G_d2_overrun,...
    G_d2_driving,brake_distribution,max_braking_torque,gamma,p_i,Fx_parameters,Fy_parameters);

%%

for i = 1:size(car_cell,1)
    car = car_cell{i,1};
    radius = 8.5;
    [x_table_skid,max_vel_skid,skidpad_time] = max_skidpad_vel(radius,car);
    time(i) = skidpad_time;
    alpha_front(i) = (x_table_skid.alpha_1+x_table_skid.alpha_2)/2;
    alpha_rear(i) = (x_table_skid.alpha_3+x_table_skid.alpha_4)/2;
    steer_angle(i) = x_table_skid.steer_angle;
    
    Fz_1(i) = x_table_skid.Fz_1;
    Fz_2(i) = x_table_skid.Fz_2;
    Fz_3(i) = x_table_skid.Fz_3;
    Fz_4(i) = x_table_skid.Fz_4;
end

plot(R_sf,time);
xlabel('LLTD','FontSize',15);
ylabel('Skidpad Time (s)','FontSize',15);
figure
plot(R_sf,alpha_front);
xlabel('LLTD','FontSize',15);
ylabel('Slip Angle (deg)','FontSize',15);
hold on
plot(R_sf,alpha_rear);
legend('Front Average','Rear Average')
figure
plot(R_sf,steer_angle);
xlabel('LLTD','FontSize',15);
ylabel('Steer Angle (deg)','FontSize',15);
figure
plot(R_sf,Fz_1);
hold on
plot(R_sf,Fz_2);
plot(R_sf,Fz_3);
plot(R_sf,Fz_4);
xlabel('LLTD','FontSize',15);
ylabel('Tire Normal Forces (lb)','FontSize',15);
legend('Front Left','Front Right', 'Rear Left','Rear ight');
