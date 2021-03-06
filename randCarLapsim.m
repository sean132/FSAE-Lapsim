% randCarLapsim2
% This generates cars with random parameters (within predefined bounds) and
% evaluates them. This can be used for sensitivities (tight bounds 
% corresponding to parameter variation) or general design exploration
% (large ranges over parameters and see what does well)
clear
setup_paths
tic
numCars = 3;
rng(0);
carArr = [];
time = struct(); time.prev = 0; time.curr = 0;
fprintf("randCar started\n");
for i = 1:numCars
    try
        carCell = randCarGen();
        car = carCell{1,1}
        accelCar = carCell{1,2};
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
    catch err
        disp(err);
    end
    carArr = [carArr car];
    save('randLapsim.mat');
end
    
    
fprintf("done\n");