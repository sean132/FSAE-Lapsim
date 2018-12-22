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
%%
clear
car = testCar();
paramArr = gg2(car);
car = makeGG(paramArr,car);
car.plotGG();
%%
clear
car = testCar();
tic
[vel_matrix_accel,vel_matrix_braking,x_table_ss,x_table_accel,x_table_braking] = g_g_diagram(car);
toc
% set desired plots to 1
    plot1 = 1; % velocity-dependent g-g diagram scatter plot
    plot2 = 1; % velocity-dependent g-g diagram surface
    plot3 = 1; % max accel for given velocity and lateral g
    plot4 = 1; % max braking for given velocity and lateral g
    plot5 = 0; % scattered interpolant version of plot3
    plot6 = 0; % scattered interpolant version of plot4
    plot7 = 1; % 2D g-g diagram for velocity specified below
    
    g_g_vel = 10; % can input vector to overlay different velocities
    
    plot_choice = [plot1 plot2 plot3 plot4 plot5 plot6 plot7];
    plotter(vel_matrix_accel,vel_matrix_braking,car.max_vel,g_g_vel,plot_choice);
    view([1 1 1]);