%%
clear; clc
hw = 10; hcg = 2; ht = 10;
k = 1; c = .02;
Ixx = 1;
Iyy = 1;
t1F = @(theta,phi) [cos(theta); 0; -sin(theta)];
t2F = @(theta,phi) [sin(theta)*sin(phi); cos(phi); cos(theta)*sin(phi)];
t3F = @(theta,phi) [sin(theta)*cos(phi); -sin(phi); cos(phi)*cos(theta)];

t1DF = @(t,td,p,pd) [-td*sin(t); 0; -td*cos(t)];
t2DF = @(t,td,p,pd) [(td*cos(t)*sin(p) + pd*sin(t)*cos(p));
                     -pd*sin(p); %there's probably a mistake in here
                     (-td*sin(t)*sin(p) + pd*cos(t)*cos(p))];
r1 = @(t,p) (hw-hcg)*t1F(t,p) + t2F(t,p)*ht/2;
r2 = @(t,p) (hw-hcg)*t1F(t,p) - t2F(t,p)*ht/2;
r3 = @(t,p) -hcg*t1F(t,p) + t2F(t,p)*ht/2;
r4 = @(t,p) -hcg*t1F(t,p) - t2F(t,p)*ht/2;

r1d = @(t,td,p,pd) (hw-hcg)*t1DF(t,td,p,pd) + t2DF(t,td,p,pd)*ht/2;
r2d = @(t,td,p,pd) (hw-hcg)*t1DF(t,td,p,pd) - t2DF(t,td,p,pd)*ht/2;
r3d = @(t,td,p,pd) -hcg*t1DF(t,td,p,pd) + t2DF(t,td,p,pd)*ht/2;
r4d = @(t,td,p,pd) -hcg*t1DF(t,td,p,pd) - t2DF(t,td,p,pd)*ht/2;

n = 1000;
t0 = 0;
p0 = pi/3;
z = [zeros(4,n-1)]; zd = zeros(4,n); 
zcgdd = zeros(4,n);
Fxap = zeros(4,n); Fyap = zeros(4,n); Fzap = zeros(4,n); %forces applied at tires
theta = [t0 zeros(1,n-1)]; thetad = zeros(1,n); thetadd = zeros(1,n);
phi = [p0 zeros(1,n-1)]; phid = zeros(1,n); phidd = zeros(1,n);
dt = .01;
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
    for j = 1:4 %moments for all 4 tires
        Fj = [Fxap(j,i-1); Fyap(j,i-1); Fzt(j)];
        momentSum = momentSum + cross(r(:,j),Fj);
    end
    phidd(i-1) = (1/Ixx)*(momentSum(1) + phid(i-1)*thetad(i-1)*sin(theta(i-1)))/cos(theta(i-1));
    thetadd(i-1) = (1/Iyy)*momentSum(2);
    %second phidd eqn should be redundant since psi set to zero
    phidd2 = -(momentSum(3) - phid(i-1)*thetad(i-1)*cos(theta(i-1)));
    %state evolution for angles
    phid(i) = phid(i-1) + dt*phidd(i-1);
    phi(i) = phi(i-1) + dt*phid(i-1);
    thetad(i) = thetad(i-1) + dt*thetadd(i-1);
    theta(i) = theta(i-1) + dt*thetad(i-1);   
end
figure(456);clf
plot(z(1,:),'o-')
hold on
plot(z(2,:),'.-')
plot(z(3,:),'o-')
plot(z(4,:),'.-')
plot(theta,'.-')
legend('z1','z2','z3','z4','theta');
disp('done');
%% rotations
clear
psiDeg = 0;     %3
thetaDeg = 0;   %2
phiDeg = 0;     %1
psi = deg2rad(psiDeg);
theta = deg2rad(thetaDeg);
phi = deg2rad(phiDeg);
x = [1;0;0];
rotMat(0,0,0,x)
%% 
clear; clc
c1 = 1; c2 = c1;
k1 = 1; k2 = k1;
I = 1;
n = 2000;
x1 = [zeros(1,n-1)]; x1d = zeros(1,n); x1dd = zeros(1,n);
x2 = zeros(1,n); x2d = zeros(1,n); x2dd = zeros(1,n);
xcg = [0 zeros(1,n-1)]; xcgd = zeros(1,n); xcgdd = zeros(1,n);
phi = [pi/10 zeros(1,n-1)]; phid = zeros(1,n); phidd = zeros(1,n);
Fz1 = [0 0 zeros(1,n-2)]; Fz2 = zeros(1,n);
Fy1 = zeros(1,n); Fy2 = zeros(1,n);
m=1; hr = .2; hw = 1; ht = 1;

dt = .01;
for i = 2:n
    x1(i-1) =   (ht/2)*sin(phi(i-1));
    x2(i-1) =    -(ht/2)*sin(phi(i-1));
    x1d(i-1) =  (ht/2)*phid(i-1)*cos(phi(i-1));
    x2d(i-1) =   -(ht/2)*phid(i-1)*cos(phi(i-1));
    xcgdd(i-1) = (1/m)*(-Fz1(i-1)+k1*x1(i-1)+c1*x1d(i-1)-Fz2(i-1)+k2*x2(i-1)+c2*x2d(i-1));
    phidd(i-1) = (1/I)*(Fy1(i-1)*hr+Fy2(i-1)*hr-Fz2(i-1)*ht+(k2*x2(i-1)+c2*x2d(i-1))*ht);
%     x1F = @(i) xcg(i) - (ht/2)*sin(phi(i));
%     x1DF = @(i) xcgd(i) - (ht/2)*phid(i)*cos(phi(i));
%     x2F = @(i) xcg(i) + (ht/2)*sin(phi(i));
%     x2DF = @(i) xcgd(i) + (ht/2)*phid(i)*cos(phi(i));
    xcgd(i) = xcgd(i-1) + dt*xcgdd(i-1);
    xcg(i) = xcg(i-1) + dt*xcgd(i-1);
    phid(i) = phid(i-1) + dt*phidd(i-1);
    phi(i) = phi(i-1) + dt*phid(i-1);
end
figure(123); clf
plot(phi,'.-')
hold on
plot(x1,'.-')
plot(x2,'.-')
plot(xcg,'.-')
legend('phi','x1','x2');
grid
disp('done');


%% 
c1=-.001;
c2 = c1;
k=100;
ht=3;
hr = .5;
dt=.001;
n = 1000;
x1 = [ [-1;0] zeros(2,n-1)];
x2 = zeros(2,n);

x1dot = @(x1,Fz1,Fz2,Fy1,Fy2) -k*x1 + Fz1 +Fz2 +((Fy1+Fy2)*hr-Fz2*ht)/ht;
x2dot = @(x2,Fz2,Fz1,x1,x1dot) -k*x2+Fz2+Fz1-k*x1-c1*x1dot;
Fz1 = [100 zeros(1,n-1)];
Fz2 = zeros(1,n);
Fy1 = zeros(1,n);
Fy2 = zeros(1,n);
for i = 2:n
    x1d = x1dot(x1(1,i-1),Fz1(i-1),Fz2(i-1),Fy1(i-1),Fy2(i-1));
    x1(1,i) = x1(1,i-1) + x1d*dt;
    x1(2,i) = x1d;
    x2d = x2dot(x2(1,i-1),Fz2(i-1),Fz1(i-1),x1(1,i-1),x1(2,i-1));
    x2(1,i) = x2(1,i-1) + x2d*dt;
    x2(2,i) = x2d;
end
figure(123);clf;
plot(x1(1,:))