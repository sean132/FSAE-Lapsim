classdef Aero
    % aero forces
    % update to return aero forces on front axle, rear axle
    
    properties
        cda
        cla
        D_f
        D_r
    end
    properties (Constant)
        rho = 1.2
    end
    
    methods
        function obj = Aero(cda,cla,distribution)
            obj.cda = cda;
            obj.cla = cla;
            obj.D_f = distribution;
            obj.D_r = 1-distribution;
        end
        
        function out = lift(obj,long_vel)            
            out = obj.rho/2*(long_vel^2)*obj.cla;
        end
        
        function out = drag(obj,long_vel)
            out = obj.rho/2*(long_vel^2)*obj.cda;
        end
    end
    
end

