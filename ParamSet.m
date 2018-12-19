classdef ParamSet
    properties
        exitflag
        longAccel
        latAccel
        steerAngle
        throttle
        longVel
        latVel
        yaw_rate
        kappa
        omega
        rpm
        gear
        beta
        Fz
        alpha
        T
        car
        longVelo
        maxLatxss
        maxLatLatAccel
        maxLatLongAccel
        maxLatX0
    end
    methods
        function obj = ParamSet(inputCar)
            if nargin > 0
                obj.car = inputCar;
                obj.longVelo = 0;
            end
        end
        function obj = setMaxLatParams(obj,vLong,x_ss,latAccel,longAccel,x0)
            %long velocity calculated at
            if obj.longVelo ~= vLong && obj.longVelo ~= 0
                error('input velocity differs from current')
            end
            obj.longVelo = vLong;
            %steady state state vector
            obj.maxLatx_ss = x_ss;
            %accelerations
            obj.maxLatLatAccel = latAccel;
            obj.maxLatLongAccel = longAccel;
            obj.maxLatX0 = x0;
        end
    end
end
    