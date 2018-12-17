%% Parameters
clearvars;clc;
% input car parameters here
% input parameters you want to iterate through as vectors


% changes to make: 
% update points computation
% differential/spool improvement
% caster jacking effect
% kinematic camber effect

% general parameters (all in metric: kg, m, etc.)
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
aeroParams.cla = 2;
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
[car_cell] = parameters_loop(carParams,aeroParams,eParams,DTparams,Bparams,tireParams);
%
% iterate through different parameter sets
for i = 1:size(car_cell,1)
    % G-G Diagram Creation
    % create velocity-dependent g-g diagram
    close all;clc
    
    car = car_cell{i,1};
    accel_car = car_cell{i,2};
    [vel_matrix_accel,vel_matrix_braking,x_table_ss,x_table_accel,x_table_braking] = ...
        g_g_diagram(car);
    % Plotting
    
    % set desired plots to 1
    plot1 = 1; % velocity-dependent g-g diagram scatter plot
    plot2 = 1; % velocity-dependent g-g diagram surface
    plot3 = 1; % max accel for given velocity and lateral g
    plot4 = 1; % max braking for given velocity and lateral g
    plot5 = 0; % scattered interpolant version of plot3
    plot6 = 0; % scattered interpolant version of plot4
    plot7 = 1; % 2D g-g diagram for velocity specified below
    
    g_g_vel = 10; % can input vector to overlay different velocities
    
%     plot_choice = [plot1 plot2 plot3 plot4 plot5 plot6 plot7];
%     plotter(vel_matrix_accel,vel_matrix_braking,car.max_vel,g_g_vel,plot_choice);
    
    % Events Calculation
    % creates struct comp including car and event time, points,
    %   accelerations, etc.
    % competitions is a cell including each comp calculated
    clear comp
    
    %comp.vel_matrix_accel = vel_matrix_accel;
    %comp.vel_matrix_braking = vel_matrix_braking;
    
    % calculate events
    events = Events(car,accel_car,vel_matrix_accel,vel_matrix_braking);
    [comp.skidpad.table,comp.skidpad.vel,comp.skidpad.time] = events.Skidpad;
    [comp.accel.time,comp.accel.ending_vel,comp.accel.long_accel,comp.accel.long_vel] = events.Accel;
    [comp.autocross.long_vel,comp.autocross.long_accel,comp.autocross.lat_accel,comp.autocross.time] = ...
        events.Autocross;
    [comp.endurance.long_vel,comp.endurance.long_accel,comp.endurance.lat_accel,comp.endurance.time] = ...
        events.Endurance;    
    % max points: skidpad = 75, accel = 100, autocross = 175, endurance = 275
    [comp.skidpad.points,comp.accel.points,comp.autocross.points,comp.endurance.points] = ...
        compute_points(comp.skidpad.time,comp.accel.time,comp.autocross.time,comp.endurance.time);
    comp.total_points = comp.skidpad.points+comp.accel.points+comp.autocross.points+comp.endurance.points;
    competitions{i} = comp;
end
% Plotting

close all

% select competition number
comp = competitions{1};

figure
plot(linspace(0,comp.autocross.time,100000),comp.autocross.lat_accel);
xlabel('Time (s)')
ylabel('Lateral Acceleration (G)')
title('Autocross Lateral Acceleration')

figure
plot(linspace(0,comp.endurance.time/16,100000),comp.endurance.lat_accel);
xlabel('Time (s)')
ylabel('Lateral Acceleration (G)')
title('Endurance Lateral Acceleration')

figure
plot(linspace(0,comp.autocross.time,100000),comp.autocross.long_accel);
xlabel('Time (s)')
ylabel('Longitudinal Acceleration (G)')
title('Autocross Longitudinal Acceleration')

figure
plot(linspace(0,comp.endurance.time/16,100000),comp.endurance.long_accel);
xlabel('Time (s)')
ylabel('Longitudinal Acceleration (G)')
title('Endurance Longitudinal Acceleration')

figure
plot(linspace(0,comp.autocross.time,100000),comp.autocross.long_vel);
xlabel('Time (s)')
ylabel('Longitudinal Velocity (m/s)')
title('Autocross Longitudinal Velocity')

figure
plot(linspace(0,comp.endurance.time/16,100000),comp.endurance.long_vel);
xlabel('Time (s)')
ylabel('Longitudinal Velocity (m/s)')
title('Endurance Longitudinal Velociy')

%% CLA Sweep
for i = 1:numel(competitions)
    comp_total_points(i) = competitions{i}.total_points;
    
    comp_skidpad_times(i) = competitions{i}.skidpad.time;
    comp_acceleration_times(i) = competitions{i}.accel.time;
    comp_autocross_times(i) = competitions{i}.autocross.time;
    comp_endurance_times(i) = competitions{i}.endurance.time;
    
    comp_skidpad_points(i) = competitions{i}.skidpad.points;
    comp_acceleration_points(i) = competitions{i}.accel.points;
    comp_autocross_points(i) = competitions{i}.autocross.points;
    comp_endurance_points(i) = competitions{i}.endurance.points;
    
end
x = cla;
y = 'cla';

figure
plot(x,comp_total_points)
xlabel(y)
ylabel('Total Points')
p1 = polyfit(x,comp_total_points,1);

figure
plot(x,comp_skidpad_times)
xlabel(y)
ylabel('skidpad times')
p2 = polyfit(x,comp_skidpad_times,1);
pp2 = polyfit(x,comp_skidpad_points,1);

figure
plot(x,comp_acceleration_times)
xlabel(y)
ylabel('Acceleration times')
p3 = polyfit(x,comp_acceleration_times,1);
pp3 = polyfit(x,comp_acceleration_points,1);

figure
plot(x,comp_autocross_times)
xlabel(y)
ylabel('Autocross times')
p4 = polyfit(x,comp_autocross_times,1);
pp4 = polyfit(x,comp_autocross_points,1);

figure
plot(x,comp_endurance_times)
xlabel(y)
ylabel('Endurance times')
p5 = polyfit(x,comp_endurance_times,1);
pp5 = polyfit(x,comp_endurance_points,1);

figure
plot(x,comp_skidpad_points)
xlabel(y)
ylabel('Skidpad points')
p5 = polyfit(x,comp_endurance_times,1);
pp5 = polyfit(x,comp_endurance_points,1);

figure
plot(x,comp_autocross_points)
xlabel(y)
ylabel('Autocross points')
p5 = polyfit(x,comp_endurance_times,1);
pp5 = polyfit(x,comp_endurance_points,1);

figure
plot(x,comp_acceleration_points)
xlabel(y)
ylabel('accel points')
p5 = polyfit(x,comp_endurance_times,1);
pp5 = polyfit(x,comp_endurance_points,1);

figure
plot(x,comp_endurance_points)
xlabel(y)
ylabel('endurance points')
p5 = polyfit(x,comp_endurance_times,1);
pp5 = polyfit(x,comp_endurance_points,1);


names = {'Points';'skidpad(s)';'Acceleration(s)';'Autocross(s)';'Endurance(s)'};
slope = [p1(1); p2(1); p3(1); p4(1); p5(1)];
points = [pp2(1) + pp3(1)+ pp4(1)+ pp5(1)  ; pp2(1); pp3(1); pp4(1); pp5(1)];
Values = table(names,slope,points)
