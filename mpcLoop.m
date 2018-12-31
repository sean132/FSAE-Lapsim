clear; clc
setup_paths
car = testCar();
car.k = 100*4.45*39.37;
car.c = 500;
car.Ixx = 60;
car.Iyy = 82;
car.TSmpc = .005;
car.TSdyn = .001;
car.Jm = 0; car.Jw = 1;
n = 2000;
%STILL USING STEADY STATE FORCES

phiArr = zeros(1,n);
phidArr = zeros(1,n);
thetaArr = zeros(1,n);
thetadArr = zeros(1,n);
FArr = zeros(4,n,3); %3 components, n steps, 4 tires
xArr = zeros(14,n);
% steer = zeros(1,n);
steer = [-.01*ones(1,n/2) .05*ones(1,n/2)];
% steer = .1*sin(linspace(0,2*pi,n));
% throttle = [.1*ones(1,800) zeros(1,200)];
throttle = zeros(1,n);
uArr = [steer; throttle];

x0 = zeros(14,1);
v = 20;
x0(3) = v; %v0 = 10 m/s
x0([8 10 12 14]) = v/car.R; %wheels at 10 m/s
xArr(:,1) = x0;
dt = car.TSmpc;
for i = 2:n
    fprintf("%d\n",i);
    [xdot, forces] = car.dynamics(xArr(:,i-1),uArr(:,i-1)); %add pitch/roll input for aero
    xArr(:,i) = xArr(:,i-1) + dt*xdot; %advance state
    FArr(:,i-1,:) =  forces.Ftotal; %store total applied forces
    %calc new pitch,roll
    state = [thetaArr(i-1); thetadArr(i-1); phiArr(i-1); phidArr(i-1)];
    [outputs,debugInfo] = calcAngles(car,state,FArr(:,i-1,:));
    thetaArr(i) = outputs.theta;
    thetadArr(i) = outputs.thetad;
    phiArr(i) = outputs.phi;
    phidArr(i) = outputs.phid;
end
ic = 1000;
figure(123);clf
plot(xArr(5,:),xArr(6,:));
hold on
plot(xArr(5,ic),xArr(6,ic),'o');
title('XY Pos');grid
xlabel('X Position');
ylabel('Y Position');
figure(456);clf
plot(xArr(3,:));
title('speed');grid
figure(789);clf
plot(rad2deg(phiArr)); hold on
plot(rad2deg(thetaArr));grid
legend('phi','theta','Location','best');
figure(1);clf
plot(rad2deg(xArr(1,:)));grid
legend('psi','Location','best');
grid
figure(2);clf
plot(rad2deg(steer)) %deg
title('steering angle');grid
figure(3); clf
plot(xArr(4,:));grid
title('lat velocity');
disp('done');