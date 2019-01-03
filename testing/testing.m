%% dynamics model testing
clear; clc
car = testCar();
v = 10;
r = car.R;
x1 = [0;
      0;
      v;
      0;
      0;
      0;
      0;
      v/r;
      0;
      v/r;
      0;
      v/r;
      0;
      v/r];
u = [1;0];
dt = .005;
car.Jm = 0; car.Jw = 1;
steps = 800;
forcesInit = struct();
forcesInit.Ftires(:,3) = car.M*car.g*ones(4,1);
forcesInit.F = zeros(1,6);
Gr = 10.6;
forcesArr = cell(steps);
xArr = zeros(numel(x1),steps);
for i = 1:steps
    [forces2, Gr] = car.calcForces(x1,u,forcesInit);
    forces = car.calcTireForces(x1,u,forces2);
    x1dot = car.dynamics(x1,u,forces,Gr);
    forcesArr{i} = forces;
%     car.printState(x1,x1dot)
    x2 = x1 + dt*x1dot;
    xArr(:,i) = x1;
    x1 = x2;
end
figure(1); clf
toPlot = [1 2];
names = {'yaw angle','yaw rate','long velo','lat velo','Xcg','Ycg','FL theta'...
    'FL w','FR theta','FR w','RL theta','RL w','RR theta','RR w'};
c1 = 1;
for i = toPlot
    subplot(numel(toPlot),1,c1)
    plot(rad2deg(xArr(i,:)),'.-');
    title(names{i});
    c1 = c1+1;
end
figure(2);clf
plot(xArr(5,:),xArr(6,:),'.-')
hold on
xlabel('xPos');ylabel('yPos');
grid
figure(3);clf
for i = 1:steps
    forces = forcesArr{i};
    plot(i,forces.alpha,'.');
    hold on
end
figure(456);clf
for i =1:steps
    
    plot(i,forces.Ftires(:,2),'.');
    hold on
end
figure(3); title('alpha');
figure(456); title('Fy');
disp('done');
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