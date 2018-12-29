function [angles, debugInfo] = calcAngles(car,state, Fap)
%Fap: 4x3 matrix: 
   %rows: 4 tires
   %columns: 3 components
hw = car.W_b;
ht = car.t_f; %front/rear trackwidth assumed same
hcg = car.l_r;
k = car.k;
m = car.M;
c = car.c;
TSmpc = car.TSmpc;
TSdyn = car.TSdyn;
Ixx = car.Ixx;
Iyy = car.Iyy;

t0 = state.theta;
p0 = state.phi;

n = round(TSmpc/TSdyn,0);
dt = TSdyn;
%unpack Fap matrix and duplicate over timesteps

% applied forces approximated as constant over dynamics
Fxap = Fap(:,1)*ones(1,n); 
Fyap = Fap(:,2)*ones(1,n); 
Fzap = Fap(:,3)*ones(1,n); 

%state arrays
theta = [t0 zeros(1,n-1)]; thetad = zeros(1,n); thetadd = zeros(1,n);
phi = [p0 zeros(1,n-1)]; phid = zeros(1,n); phidd = zeros(1,n);
% zcgdd = zeros(4,n); %not currently tracking zcgdd, need to
z = zeros(4,n); zd = zeros(4,n); 

%basis vectors, 321 euler angle, psi = 0
t1F = @(theta,phi) [cos(theta); 0; -sin(theta)];
t2F = @(theta,phi) [sin(theta)*sin(phi); cos(phi); cos(theta)*sin(phi)];
t3F = @(theta,phi) [sin(theta)*cos(phi); -sin(phi); cos(phi)*cos(theta)];
%basis vector derivatives, 321 euler angles, psi = 0
t1DF = @(t,td,p,pd) [-td*sin(t); 0; -td*cos(t)];
t2DF = @(t,td,p,pd) [(td*cos(t)*sin(p) + pd*sin(t)*cos(p));
                     -pd*sin(p); 
                     (-td*sin(t)*sin(p) + pd*cos(t)*cos(p))];
%tire position vectors (3x1 vector)
r1 = @(t,p) (hw-hcg)*t1F(t,p) + t2F(t,p)*ht/2;
r2 = @(t,p) (hw-hcg)*t1F(t,p) - t2F(t,p)*ht/2;
r3 = @(t,p) -hcg*t1F(t,p) + t2F(t,p)*ht/2;
r4 = @(t,p) -hcg*t1F(t,p) - t2F(t,p)*ht/2;
%tire position vector derivatives (3x1 vector)
r1d = @(t,td,p,pd) (hw-hcg)*t1DF(t,td,p,pd) + t2DF(t,td,p,pd)*ht/2;
r2d = @(t,td,p,pd) (hw-hcg)*t1DF(t,td,p,pd) - t2DF(t,td,p,pd)*ht/2;
r3d = @(t,td,p,pd) -hcg*t1DF(t,td,p,pd) + t2DF(t,td,p,pd)*ht/2;
r4d = @(t,td,p,pd) -hcg*t1DF(t,td,p,pd) - t2DF(t,td,p,pd)*ht/2;

%oscillation amplitude decreases with step size
for i = 2:n
    tc = theta(i-1); tcd = thetad(i-1);
    pc = phi(i-1); pcd = phid(i-1);
    r = [r1(tc,pc) r2(tc,pc) r3(tc,pc) r4(tc,pc)];
    z(:,i-1) = r(3,:)'; %take z components and store them
    rd = [r1d(tc,tcd,pc,pcd) r2d(tc,tcd,pc,pcd) r3d(tc,tcd,pc,pcd) r4d(tc,tcd,pc,pcd)];
    zd(:,i-1) = rd(3,:)'; %take z velocities and store them
    
    Fzs = -k*z(:,i-1) - c*zd(:,i-1); %forces from shocks, positive z: spring in tension
    Fzt = Fzs + Fzap(:,i-1); %Ftotal: add to applied z forces
    momentSum = 0;
    for j = 1:4 %moments for all 4 tires; M = rxF
        Fj = [Fxap(j,i-1); Fyap(j,i-1); Fzt(j)];
        momentSum = momentSum + cross(r(:,j),Fj);
    end
    phidd(i-1) = (1/Ixx)*(momentSum(1) + phid(i-1)*thetad(i-1)*sin(theta(i-1)))/cos(theta(i-1));
    thetadd(i-1) = (1/Iyy)*momentSum(2);
    %second phidd eqn should be redundant since psi set to zero. currently
    %all zeros,need to figure out why
%     phidd2(i-1) = -(momentSum(3) - phid(i-1)*thetad(i-1)*cos(theta(i-1)));
    %state evolution for angles
    phid(i) = phid(i-1) + dt*phidd(i-1);
    phi(i) = phi(i-1) + dt*phid(i-1);
    thetad(i) = thetad(i-1) + dt*thetadd(i-1);
    theta(i) = theta(i-1) + dt*thetad(i-1);   
end
t = (1:n).*dt;
angles = struct();
angles.theta = theta(n);
angles.thetad = thetad(n);
angles.phi = phi(n);
angles.phid = phid(n);
debugInfo = struct();
debugInfo.theta = theta;
debugInfo.thetad = thetad;
debugInfo.phi = phi;
debugInfo.phid = phid;
debugInfo.z = z;
debugInfo.t = t;
debugInfo.zd = zd;