%% Parameters
clearvars;clc;
% input car parameters here
% input parameters you want to iterate through as vectors

% general parameters (all in metric: kg, m, etc.)
mass = 166.967; %170.097; % not including driver
driver_weight = 72.5748;
accel_driver_weight = 68.0389;
wheelbase = 1.524;
weight_dist = 0.51; % percentage of weight in rear
track_width = 1.1938;
wheel_radius = 0.1956; %0.221; %0.232; % should ideally be loaded radius
cg_height = 0.2794;
roll_center_height_front = 0.052;
roll_center_height_rear = 0.0762;
R_sf = 0.4; %0.3:0.01:0.6; % proportion of roll stiffness in front (not same as LLTD)
I_zz = 83.28; %kg-m^2

% aero parameters
cda = 0.9;
cla = 1.771;
distribution = 0.5; % proportion of downforce in front

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
load('Fy_combined_parameters_run38_new.mat'); % F_y combined magic formula parameters
Fy_parameters = cell2mat(Xbestcell);

[car_cell] = parameters_loop(mass,driver_weight,accel_driver_weight,wheelbase,weight_dist,track_width,...
    wheel_radius,cg_height,roll_center_height_front,roll_center_height_rear,R_sf,I_zz,cda,cla,...
    distribution,redline,shift_point,gears,torque_fn,final_drive,drivetrain_efficiency,G_d1,G_d2_overrun,...
    G_d2_driving,brake_distribution,max_braking_torque,gamma,p_i,Fx_parameters,Fy_parameters);

car = car_cell{1,1};
    
%%
counter = 1;
radius = 8.625;

[x_table, x_guess] = constant_radius(radius,4,car);
x0 = x_guess;

long_vel_guess = sqrt(1.5*radius*9.81);
[~, max_vel_skid, ~] = max_skidpad_vel(radius,long_vel_guess,car,x0);

velocities = 3:0.1:max_vel_skid-0.5;
steer_angle = zeros(size(velocities));
beta = zeros(size(velocities));
lat_accel = zeros(size(velocities));
long_vel = zeros(size(velocities));
yaw_rate = zeros(size(velocities));

for i = velocities
    x0(3) = i;
    x0(5) = i/radius;
    
    [x_table, x_guess] = constant_radius(radius,i,car,x0);
    x0 = x_guess;
    steer_angle(counter) = x_table{1,'steer_angle'};
    beta(counter) = x_table{1,'beta'};
    lat_accel(counter) = x_table{1,'lat_accel'};
    long_vel(counter) = x_table{1,'long_vel'};
    yaw_rate(counter) = x_table{1,'yaw_rate'};
    counter = counter+1;
end

%% Measured Data Plotting

% note that steer_angle is the angle of the front wheels, not the steering wheel
% also called "reference steer angle"
% 
% figure
% plot([-fliplr(lat_accel/9.81) lat_accel/9.81],[fliplr(steer_angle) steer_angle])
% xlabel('Lateral Accel (g)')
% ylabel('Steer Angle (deg)')
% 
% figure
% plot([-fliplr(lat_accel/9.81) lat_accel/9.81],[fliplr(beta) beta])
% xlabel('Lateral Accel (g)')
% ylabel('Sideslip Angle (deg)')
 
figure
plot(lat_accel/9.81,steer_angle)
xlabel('Lateral Accel (g)')
ylabel('Steer Angle (deg)')

%% Gradient Calculations

K = diff(steer_angle)./diff(lat_accel/9.81);
figure
%plot([-fliplr(lat_accel(2:end)/9.81) lat_accel(2:end)/9.81], [fliplr(K) K]);
plot(lat_accel(2:end)/9.81, K);
hold on
xlabel('Lateral Accel (g)','FontSize',15)
ylabel('Understeer Gradient (deg/g)','FontSize',15)

%title('Understeer Gradient Sensitivity to LLTD','FontSize',18)
%legend('LLTD = 0.3', 'LLTD = 0.4', 'LLTD = 0.5', 'LLTD = 0.6')

% sideslip_gradient = diff(beta)./diff(lat_accel/9.81);
% figure
% plot([-fliplr(lat_accel(2:end)/9.81) lat_accel(2:end)/9.81], [fliplr(sideslip_gradient) sideslip_gradient]);
% xlabel('Lateral Accel (g)')
% ylabel('Sideslip Angle Gradient (deg/g)')
%% Steering Ratio Calculations
% 
% steer = [0.1 (2.597+2.627)/2 
%     0.2 (5.171+5.292)/2
%     0.3 (7.727+8.003)/2
%     0.4 (10.271+10.769)/2
%     0.5 (13.603+12.808)/2
%     0.6 (15.343+16.517)/2
%     0.7 (17.881+19.530)/2
%     0.8 (22.662+20.427)/2
%     0.9 (22.986+25.946)/2
%     ];
% 
% % 0.00938 in/deg rack travel
% 
% % deg steering wheel angle / avg steer angle of front wheels
% steering_ratio = mean(steer(:,1)./steer(:,2)/0.00938);
% 
% 
% %% Characteristic/Critical Speed
% 
% V_char = real(sqrt(57.3*car.W_b*9.81./K))
% V_crit = real(sqrt(-57.3*car.W_b*9.81./K))
% 
% plot(V_char)
% ylim([0 200])
% figure
% plot(V_crit)
% ylim([0 200])
% 
% %%
% figure
% plot(long_vel,yaw_rate./(steer_angle*pi/180))
% xlabel('velocity (m/s)')
% ylabel('r/delta (1/s)')
% title('Yaw Velocity Gain')
% 
% figure
% plot(long_vel,lat_accel./steer_angle)
% xlabel('velocity (m/s)')
% ylabel('Ay/delta (m/s^2/deg)')
% title('Lateral Acceleration Gain')
