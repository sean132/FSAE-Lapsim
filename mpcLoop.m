clear; clc
setup_paths
car = testCar();
car.k = 100*4.45*39.37;
car.c = 40;
car.Ixx = 60;
car.Iyy = 82;
car.TSmpc = .003;
car.TSdyn = .001;
car.Jm = 0; car.Jw = 1;
n = 16000;

phiArr = zeros(1,n);
phidArr = zeros(1,n);
thetaArr = zeros(1,n);
thetadArr = zeros(1,n);
zArr = [car.h_rc zeros(1,n-1)]; %rotation axis height
zdArr = zeros(1,n);

xArr = zeros(14,n);
% steer = zeros(1,n);
steerDeg = 20;
steer = deg2rad(steerDeg)*[ones(1,n/4) ones(1,2*n/4) ones(1,n/4)];
% throttle = zeros(1,n);
throttle = 0*ones(1,n);
uArr = [steer; throttle];

x0 = zeros(14,1);
v = 20;
x0(3) = v; %v0 = 10 m/s
x0([8 10 12 14]) = v/car.R; %wheels at 10 m/s
xArr(:,1) = x0;
dt = car.TSmpc;
g = 9.81;

FapTotal = cell(n); %applied forces at each step
% Fxyz = [0 10 0];
% Rxyz = [0 0 car.R];
Rxyz = [0 0 car.h_g];
Fg = [0 0 -car.M*g];
% Fg = [0 0 0];
Rg = [-car.l_f 0 car.h_rc];
Fconstant = [Fg Rg]; %1000 N upward at front axle
for i = 1:n
    forces = struct();
    forces.T = 0;
    forces.F = Fconstant;
    forces.Ftires = zeros(4,6);
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
    if i == 1000
        disp('hold');
    end
%     fprintf("%d\n",i);
    angles = [thetaArr(i-1); thetadArr(i-1); 
              phiArr(i-1); phidArr(i-1); 
              zArr(i-1); zdArr(i-1)];
    forces1 = FapTotal{i-1};
    x = xArr(:,i-1);
    u = uArr(:,i-1);
    
    %calculate forces at arbitrary locations
    %store them in populate forces.F
    [forces2, Gr] = car.calcForces(x,u,forces1);
    %forces at t = i-1
    
    %calculate forces produced by the tires
    % appends XYZ forces from tires to forces.F
    % positive Z forces balance applied Z forces
    forces3 = car.calcTireForces(x,u,forces2);
    %uses Ftires 
    
    
    %resolve forces to xyz forces at tires. 
    %  -> Tire model needs z forces at tires
    %calculate new  theta/phi (outputs struct)
    %populate forces.Ftires
    %forces.Ftires is the z component of the force on each tire
    %equal in magnitude to the force produced by the shock
    % does not append to forces.F
    [outputs,forces4,nextFz] = calcAngles(car,x,angles,forces3);
    %outputs Ftires at TS = i
    
    % applies forces.F to the car to produce xdot
    [xdot, forces5] = car.dynamics(x,u,forces4,Gr);
    %xdot at i-1
    
    %advance state
    xArr(:,i) = xArr(:,i-1) + dt*xdot; 
    fprintf("%0.2f\n",xdot(4));
    %store new pitch,roll
    thetaArr(i) = outputs.theta;
    thetadArr(i) = outputs.thetad;
    phiArr(i) = outputs.phi;
    phidArr(i) = outputs.phid;
    zArr(i) = outputs.z;
    zdArr(i) = outputs.zd;
    FapTotal{i-1} = forces5; %store all applied forces
    nextF = FapTotal{i};
    nextF.Ftires(:,3) = nextFz;
    FapTotal{i} = nextF;
    if i == 1000
        disp('hold');
    end
end
ic = 1000;
figure(123);
plot(xArr(5,:),xArr(6,:));
hold on
plot(xArr(5,ic),xArr(6,ic),'o');
title('XY Pos');grid
xlabel('X Position');
ylabel('Y Position');
figure(456);clf
plot(sqrt(xArr(3,:).^2 + xArr(4,:).^2));
title('speed');grid
figure(789);
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
figure(3);
plot(xArr(4,:));grid;hold on
title('lat velocity');
disp('done');
fprintf("phi: %0.2f\n",rad2deg(phiArr(end-10)));
fprintf("theta: %0.2f\n",rad2deg(thetaArr(end-10)));