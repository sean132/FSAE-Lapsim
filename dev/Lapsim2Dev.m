% Lapsim2
clear
setup_paths
carCell = carConfig(); %generate all cars to sim over
numCars = size(carCell,1);
time = struct();time.prev = 0; time.curr = 0;
tic
disp('The parallel toolbox takes a few minutes to start.')
for i = 1:numCars
    car = carCell{i,1};
    accelCar = carCell{i,2};
    fprintf("car %d of %d - starting g-g\n",[i numCars]);
    paramArr = gg2(car);
    fprintf("car %d of %d - g-g complete\n",[i numCars]);
    time.curr = floor(toc);
    fprintf("Stage Time: %d s; Total time elapsed: %d s\n",[time.curr-time.prev time.curr]);
    time.prev = time.curr;
    car = makeGG(paramArr,car); %post-processes gg data and stores in car
    comp = Events2(car,accelCar); 
    comp.calcTimes();       %run events and calc points
    car.comp = comp;        %store in array
    fprintf("car %d of %d - points calculated\n",[i numCars]);
    time.curr = floor(toc);
    fprintf("Stage Time: %d s; Total time elapsed: %d s\n",[time.curr-time.prev time.curr]);
end
fprintf("done\n");
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
