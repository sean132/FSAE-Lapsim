function constr = advanceState(car,xArr,uArr,n)
    phiArr = zeros(1,n);
    phidArr = zeros(1,n);
    thetaArr = zeros(1,n);
    thetadArr = zeros(1,n);
    zArr = [car.h_rc zeros(1,n-1)]; %rotation axis height
    zdArr = zeros(1,n);
    FzArr = zeros(4,n);
    
    FapTotal = cell(n); %applied forces at each step
    Fg = [0 0 -car.M*car.g];
    Rg = [-car.l_f 0 car.h_rc];
    Fconstant = [Fg Rg]; 
    for i = 1:n
        forces = struct();
        forces.T = 0; %wheel torques
        forces.F = Fconstant; %applied forces
        forces.Ftires = zeros(4,6); %forces applied by tires
        forces.Fxw = 0;         %x forces in front wheels tire csys
        forces.Fx = 0;          %x forces, kept for compatibility
        FapTotal{i} = forces;
    end
    forceInit = FapTotal{1};
    forceInit.Ftires(:,3) = car.M*car.g*.25*ones(4,1);
    FapTotal{1} = forceInit;
    for i = 2:n
        i
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
        forces3 = car.calcTireForces(x,u,forces2);

        [outputs,forces4,nextFz] = calcAngles(car,x,angles,forces3,'yalmip');

        [xdot, forces5] = car.dynamics(x,u,forces4,Gr);
        %xdot at i-1
        %advance state
        constr = [constr; xArr(:,i) == xArr(:,i-1) + dt*xdot]; 
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