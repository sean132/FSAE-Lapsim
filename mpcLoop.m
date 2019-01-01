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

phiArr = zeros(1,n);
phidArr = zeros(1,n);
thetaArr = zeros(1,n);
thetadArr = zeros(1,n);

xArr = zeros(14,n);
steer = zeros(1,n);
% steer = [-.01*ones(1,n/2) .01*ones(1,n/2)];
% steer = .1*sin(linspace(0,2*pi,n));
% throttle = [.1*ones(1,800) zeros(1,200)];
throttle = .5*zeros(1,n);
uArr = [steer; throttle];

x0 = zeros(14,1);
v = 20;
x0(3) = v; %v0 = 10 m/s
x0([8 10 12 14]) = v/car.R; %wheels at 10 m/s
xArr(:,1) = x0;
dt = car.TSmpc;
g = 9.81;

FapTotal = cell(n); %applied forces at each step
Fconstant = [0 0 0 0 0 1]; %1000 N upward at front axle
for i = 1:n
    forces = struct();
    forces.T = 0;
    forces.F = Fconstant;
    forces.Ftires = 0;
    forces.Fxw = 0;         %x forces in front wheels tire csys
    forces.Fx = 0;
    FapTotal{i} = forces;
end
% FArr = zeros(4,3,n); %4 tires, 3 components, n steps
% Fapplied = [0 0 -car.M*g -car.l_f 0 car.h_g]; %gravity, need to add cg height
% Fapplied = zeros(1,6);

%all functions take in Fapplied and FArr
%all functions return FArr: 
% FArr(:,2,:) = 500;
for i = 2:n
    fprintf("%d\n",i);
    angles = [thetaArr(i-1); thetadArr(i-1); phiArr(i-1); phidArr(i-1)];
    forces1 = FapTotal{i-1};
    x = xArr(:,i-1);
    u = uArr(:,i-1);
    
    %calculate forces at arbitrary locations
    %store them in populate forces.F
    [forces2, Gr] = car.calcForces(x,u,forces1);
    
    %resolve forces to xyz forces at tires. 
    %  -> Tire model needs z forces at tires
    %calculate new theta/phi (outputs struct)
    %populate forces.Ftires
    %forces.Ftires is the z component of the force on each tire
    %equal in magnitude to the force produced by the shock
    % does not append to forces.F
    [outputs,forces3] = calcAngles(car,angles,forces2);
    
    %calculate forces produced by the tires
    % appends XYZ forces from tires to forces.F
    % positive Z forces balance applied Z forces
    forces4 = car.calcTireForces(x,u,forces3);
    
    % applies forces.F to the car to produce xdot
    [xdot, forces5] = car.dynamics(x,u,forces4,Gr);
    
    %advance state
    xArr(:,i) = xArr(:,i-1) + dt*xdot; 
    
    %store new pitch,roll
    thetaArr(i) = outputs.theta;
    thetadArr(i) = outputs.thetad;
    phiArr(i) = outputs.phi;
    phidArr(i) = outputs.phid;
    FapTotal{i-1} = forces5; %store all applied forces
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
title('phi and theta, deg');
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