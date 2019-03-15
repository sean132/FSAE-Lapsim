function outputs = fullDynamics(car,uArr,x0,n)

phiArr = zeros(1,n); % roll angle
phidArr = zeros(1,n); % roll velocity
thetaArr = zeros(1,n); % pitch angle
thetadArr = zeros(1,n); % pitch velocity
zArr = [car.h_rc zeros(1,n-1)]; %rotation axis height
zdArr = zeros(1,n); 
FzArr = zeros(4,n); % vertical forces on tires

xArr = zeros(14,n); % state array
xArr(:,1) = x0;
dt = car.TSmpc;
g = 9.81;

FapTotal = cell(n); %applied forces at each step
Fxyz = [0 10 0]; % applied force
Rxyz = [0 0 0];
Rxyz = [0 0 car.h_g]; % position of applied force
Fg = [0 0 -car.M*g]; % gravity
% Fg = [0 0 0];
Rg = [-car.l_f 0 car.h_g];
Fconstant = [Fg Rg]; 
for i = 1:n
    forces = struct();
    forces.T = 0; %wheel torques
    forces.F = Fconstant; %applied forces
    forces.Ftires = zeros(4,6); %forces applied by tires
    forces.Fxw = 0;         %x forces in front wheels tire csys
    forces.Fx = 0;          %x forces, in car coordinate system, kept for compatibility
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
    forces5.Ftires
    %store new pitch,roll
    thetaArr(i) = outputs.theta;
    thetadArr(i) = outputs.thetad;
    phiArr(i) = outputs.phi;
    phidArr(i) = outputs.phid;
    zArr(i) = outputs.z;
    zdArr(i) = outputs.zd;
    FzArr(:,i) = forces5.Ftires(:,3);
    
    FapTotal{i-1} = forces5; %store all applied forces
    nextF = FapTotal{i};
    nextF.Ftires(:,3) = nextFz;
    FapTotal{i} = nextF;
end
outputs = struct();
outputs.xArr = xArr;
outputs.phiArr = phiArr;
outputs.thetaArr = thetaArr;
outputs.FzArr = FzArr;
outputs.zArr = zArr;