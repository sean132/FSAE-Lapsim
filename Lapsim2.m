% Lapsim2
clear
carCell = carConfig; %generate all car to sim over

for i = 1:size(carCell,1)
    car = carCell{i,1};
    accelCar = carCell{i,2};
    paramArr = gg2(car);
    car = makeGG(paramArr,car); %post-processes gg data and stores in car
    comp = Events2(car,accelCar); 
    comp.calcTimes();       %run events and calc points
    car.comp = comp;
end