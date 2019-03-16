clear; clc
% run this from the same directory as setup_paths
try 
    setup_paths
catch
    error('run this from same directory as setup_paths. Should be one directory up');
end

% input car parameters
car = testCar();
car.k = 200*4.45*39.37; 
car.c = 700;
car.Ixx = 60;
car.Iyy = 82;
car.TSmpc = .003; %has to be multiple of TSdyn
car.TSdyn = .0005;
car.Jm = 0; car.Jw = 1;
n = 8000; % number of timesteps

% steering/throttle input
steerDeg = 3;
steer = deg2rad(steerDeg)*[zeros(1,n/8) ones(1,7*n/8)];
time = 0:car.TSmpc:car.TSmpc*(n-1);
steer = deg2rad(steerDeg)*sin((2*pi)*time);
steer(1:3000) = 0;
throttle = zeros(1,n);
%throttle = [0*ones(1,n/2) 1*ones(1,n/4) -1*ones(1,n/4)];
% throttle = [zeros(1,n/4) ones(1,2*n/4) -ones(1,n/4)];
uArr = [steer; throttle];

% initial vehicle states (vector of 14 values)
% 1: yaw angle 2: yaw rate 3: long velocity 4: lat velocity
% 5: x position of cg 6: y position of cg
% 7:  FL angular position 8:  FL angular velocity
% 9:  FR angular position 10: FR angular velocity
% 11: RL angular position 12: RL angular velocity
% 13: RR angular position 14: RR angular velocity

x0 = zeros(14,1);
v = 20; % initial velocity of 20 m/s
x0(3) = 20; 
x0([8 10 12 14]) = v/car.R; % wheel velocities

% main dynamics solver 
% outputs data: struct containing time histories of states, 
% roll/pitch angles, tire normal loads, and wheel displacements
data = fullDynamics(car,uArr,x0,n);

%% Plotting

xArr = data.xArr; FzArr = data.FzArr; phiArr = data.phiArr;
thetaArr = data.thetaArr; zArr = data.zArr;
ic = 1000;
figure(123);clf
plot(xArr(5,:),xArr(6,:));
hold on
plot(xArr(5,ic),xArr(6,ic),'o');
title('XY Pos');grid
xlabel('X Position');
ylabel('Y Position');
figure(456);clf
plot(sqrt(xArr(3,:).^2 + xArr(4,:).^2));
title('speed');grid
figure(789);clf
plot(rad2deg(phiArr)); hold on
plot(rad2deg(thetaArr));grid
title('phi and theta, deg');
legend('phi','theta','Location','best');
figure(1);clf
plot(rad2deg(xArr(1,:)));grid
legend('psi','Location','best');
grid
figure(2);clf
plot(rad2deg(steer)) %deg
title('steering angle');grid
figure(3);clf
for i =1:4
    plot(FzArr(i,:));hold on
end
grid
title('Fz'); legend('1','2','3','4');
figure(5);clf
plot(zArr);
title('CG Height');
% plot(xArr(4,:));grid;hold on
% title('lat velocity');
disp('done');
fprintf("phi: %0.2f\n",rad2deg(phiArr(end-10)));
fprintf("theta: %0.2f\n",rad2deg(thetaArr(end-10)));