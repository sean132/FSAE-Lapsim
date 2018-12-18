%% max lat accel testing
clear
car = testCar();
x0 = zeros(1,9);
vGuess = 10; %m/s

[x_accel,long_accel,long_accel_guess] = max_lat_accel(vGuess,car,x0);
% max_long_accel_cornering(vGuess,lat_accel_value,car,x0)
%%
clear
figure(1);clf
a = Polyhedron('V',[0 0 0; 
                    1 1 1; 
                    1 0 0; 
                    0 0 1; 
                    1 1 0; 
                    0 0 1;
                    0 1 1;
                    1 1 0;
                    1 0 1]);
a.plot();