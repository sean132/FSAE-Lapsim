%% max lat accel testing
clear
car = testCar();
x0 = zeros(1,9);
vGuess = 10; %m/s

[x_accel,long_accel,long_accel_guess] = max_lat_accel(vGuess,car,x0);
% max_long_accel_cornering(vGuess,lat_accel_value,car,x0)