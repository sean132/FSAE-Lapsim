function paramArr = gg2(car)
minV = 5; maxV = car.max_vel-.5;
longVgrid = 25;
latAgrid = 30;

longVelArr = linspace(minV,maxV,longVgrid);
paramArr(longVgrid,latAgrid) = ParamSet();
parfor c1 = 1:numel(longVelArr)
    longVel = longVelArr(c1);
    [maxLatx,maxLatLatAccel,maxLatLongAccel,maxLatx0] = max_lat_accel(longVel,car);
    latAccelArr = linspace(0,maxLatLatAccel,latAgrid);
    row = ParamSet();
    row(numel(latAccelArr)) = ParamSet();
    for c2 = 1:numel(latAccelArr)
        latAccelg = latAccelArr(c2);
        latAccel = latAccelg*9.81;
        
        [xAccel,longAccel,longAccelx0] = max_long_accel_cornering(longVel,...
                latAccel,car);
        [xBraking,longDecel,brakingDecelx0] = max_braking_decel_cornering(longVel,...
                latAccel,car);
%         fprintf("%0.2f %0.2f %0.2f\n",[longDecel longVel latAccel]); 
%         fprintf("%0.2f %0.2f %0.2f\n",[longVel maxLatLatAccel longAccel]);
%         fprintf("%0.2f %0.2f %0.2f\n",[longVel maxLatLatAccel longDecel]);
        carParams = ParamSet(car,longVel); 
        carParams = carParams.setMaxLatParams(maxLatx,maxLatLatAccel,maxLatLongAccel,maxLatx0);
        carParams = carParams.setMaxAccelParams(xAccel,longAccel,latAccel,longAccelx0);
        carParams = carParams.setMaxDecelParams(xBraking,longDecel,latAccel,brakingDecelx0);
        row(c2) = carParams;
    end
    paramArr(c1,:) = row;
end