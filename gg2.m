function paramArr = gg2(car)
minV = 5; maxV = car.max_vel-.5;
longVgrid = 25;
latAgrid = 30;

longVelArr = linspace(minV,maxV,longVgrid);
paramArr(longVgrid,latAgrid) = ParamSet();
for c1 = 1:numel(longVelArr)
    longVel = longVelArr(c1);
    [xss,maxLatLatAccel,maxLatLongAccel,maxLatx0] = max_lat_accel(longVel,car);
    latAccelArr = linspace(.1,maxLatLatAccel-.1,latAgrid);
    row = ParamSet();
    row(numel(latAccelArr)) = ParamSet();
    for  c2 = 1:numel(latAccelArr)
        latAccelg = latAccelArr(c2);
        latAccel = latAccelg*9.81;
        
        [xAccel,longAccel,longAccelx0] = max_long_accel_cornering(longVel,...
                latAccel,car);
        
        [xBraking,longDecel,brakingDecelx0] = max_braking_decel_cornering(longVel,...
                latAccel,car);
        carParams = ParamSet(car); 
        carParams.exitflag = c1;
        carParams.longAccel = c2;
        row(c2) = carParams;
    end
    paramArr(c1,:) = row;
end