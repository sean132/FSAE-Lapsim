% Lapsim2
clear
carCell = carConfig; %generate all car to sim over

for i = 1:size(carCell,1)
    car = carCell{i,1};
    accelCar = carCell{i,2};
    paramArr = gg2(car);
    car = makeGG(paramArr,car); %post-processes gg data and stores in car
    comp = Events2(car,accelCar); 
    comp.calcTimes();       %run events and calc points
    car.comp = comp;
end
%%
clear
load lp2.mat
comp = Events2(car,accelCar); 
comp.calcTimes();
disp('done');
%%
clear
load lp1.mat
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
