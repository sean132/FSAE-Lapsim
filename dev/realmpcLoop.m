clear; clc
setup_paths
car = testCar();
car.k = 200*4.45*39.37;
car.c = 300;
car.Ixx = 60;
car.Iyy = 82;
car.TSmpc = .003;
car.TSdyn = .0005;
car.Jm = 0; car.Jw = 1;
n = 10;

steerLim = deg2rad(15);
throttleLim = 1;


steer = sdpvar(1,n);
throttle = sdpvar(1,n);
uArr = [steer; throttle];
xArr = sdpvar(14,n);

x0 = zeros(14,1);
v = 20;
x0(3) = v; %v0 = 10 m/s
x0([8 10 12 14]) = v/car.R; %wheels at 10 m/s
dt = car.TSmpc;
g = 9.81;
constr = [];
constr = [constr; x0 == xArr(:,1)];
%input limit constraints
for i = 1:n
    constr = [constr; steer(i) <= steerLim;
                      steer(i) >= -steerLim;
                      throttle(i) <= throttleLim;
                      throttle(i) >= -throttleLim];
end
%boundary constraints
yBLim = 1; 
for i = 1:n
    constr = [constr; xArr(6,i) <= yBLim;
                      xArr(6,i) >= -yBLim;
                      xArr(3,i) >= 1];
end
%dynamics constraints
constr = [constr advanceState(car,xArr,uArr,n)];
goalXPos = 100;
obj = goalXPos-xArr(6,n);
options = sdpsettings('solver','fmincon','verbose',0);
diag = optimize(constr,obj,options);