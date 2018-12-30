clear; clc
setup_paths
car = testCar();
n = 10000;
phiArr = zeros(1,n);
phidArr = zeros(1,n);
thetaArr = zeros(1,n);
thetadArr = zeros(1,n);
xArr = zeros(14,n);
steer = zeros(1,n);
throttle = zeros(1,n);
uArr = [steer; throttle];
dt = .05;
for i = 2:n
    xArr(:,i) = xArr(:,i-1) + dt*car.dynamics(x(:,i-1),u(:,i-1));
    
end