classdef ParamSet
    properties
        car
        longVel
        maxLatFlag
        maxLatx
        maxLatLatAccel
        maxLatLongAccel
        maxLatX0
        
        maxLongFlag
        maxLongAccelx
        maxLongLongAccel
        maxLongLatAccel
        maxLongAccelx0
        
        maxBrakeFlag
        maxBrakeDecelx
        maxBrakeLongDecel
        maxBrakeLatAccel
        maxBrakingDecelx0
        
    end
    methods
        function obj = ParamSet(inputCar,longVel)
            if nargin > 0
                obj.car = inputCar;
                obj.longVel = longVel;
            end
        end
        function obj = setMaxLatParams(obj,x_ss,latAccel,longAccel,x0)
            %steady state state vector
            obj.maxLatx = x_ss;
            obj.maxLatFlag = x_ss(1);
            %accelerations
            obj.maxLatLatAccel = latAccel;
            obj.maxLatLongAccel = longAccel;
            obj.maxLatX0 = x0;
        end
        function obj = setMaxAccelParams(obj,xAccel,longAccel,latAccel,longAccelx0)
            obj.maxLongFlag = xAccel(1);
            obj.maxLongAccelx = xAccel;
            obj.maxLongLongAccel = longAccel;
            obj.maxLongLatAccel = latAccel;
            obj.maxLongAccelx0 = longAccelx0;
        end
        function obj = setMaxDecelParams(obj,xBraking,longDecel,latAccel,longDecelx0)
            obj.maxBrakeFlag = xBraking(1);
            obj.maxBrakeDecelx = xBraking;
            obj.maxBrakeLongDecel = longDecel;
            obj.maxBrakeLatAccel = latAccel;
            obj.maxBrakingDecelx0 = longDecelx0;
        end
    end
end
    