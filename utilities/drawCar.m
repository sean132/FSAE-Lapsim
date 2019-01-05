function lineHandle = drawCar(figureNum,lineHandle,car,xy,psi,i)
figure(figureNum);
rotMatr = [cos(psi) -sin(psi);
            sin(psi) cos(psi)];
points = [car.W_b/2 car.W_b/2  -car.W_b/2 -car.W_b/2; 
          car.t_f/2 -car.t_f/2 -car.t_f/2 car.t_f/2];
          
rotPoints = xy + rotMatr*points;
rotPoints = [rotPoints rotPoints(:,1)];
if i == 1
    lineHandle = line();
end
lineHandle.XData =  rotPoints(1,:);
lineHandle.YData = rotPoints(2,:);
