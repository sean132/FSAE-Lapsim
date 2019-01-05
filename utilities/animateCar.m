function animateCar(car,xArr)
xyArr = [xArr(5,:); xArr(6,:)];
psiArr = xArr(1,:);

Ts = .1; %pause time
rate = 10; %timesteps to skip
figNum = 10;
lineHandle = 0;
figure(figNum); clf
for i = 1:rate:size(xyArr,2)
    lineHandle = drawCar(figNum,lineHandle,car,xyArr(:,i),psiArr(i),i);
    xlim([-20 20]);
    ylim([-20 20]);
    axis square
    pause(Ts);
end
