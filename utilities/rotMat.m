function vec = rotMat(psi,theta,phi,x)
%rotate vector x by 321 euler angles
R3 = [cos(psi) sin(psi) 0;
      -sin(psi) cos(psi) 0;
      0           0       1];
R2 = [cos(theta) 0 -sin(theta);
     0           1    0;
     sin(theta) 0 cos(theta)];
R1 = [1 0          0;
      0 cos(phi) sin(phi);
      0 -sin(phi) cos(phi)];
mat = R1*R2*R3;
mat = mat';
vec = mat*x;