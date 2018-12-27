%%


%as
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