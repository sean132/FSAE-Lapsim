function animateCar(car,xArr)
xyArr = [xArr(5,:); xArr(6,:)];
psiArr = xArr(1,:);

rate = 5; %timesteps to skip
figNum = 123;
lineHandle = 0;
figure(figNum); clf
plot(xyArr(1,:),xyArr(2,:),'b');
xlim(1.2*[min(min(xyArr))-1 1+max(max(xyArr))]);
ylim(1.2*[min(min(xyArr))-1 1+max(max(xyArr))]);
hold on
for i = 1:rate:size(xyArr,2)
    lineHandle = drawCar(figNum,lineHandle,car,xyArr(:,i),psiArr(i),i);
    
    axis square
    pause(rate*car.TSmpc);
end
