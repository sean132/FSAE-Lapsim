classdef Powertrain
    % includes engine, drivetrain, and brakes model
    
    properties
        redline
        shift_point
        gears
        final_drive 
        torque_fn
        wheel_radius
        drivetrain_efficiency
        
        brake_distribution
        G_d1
        G_d2_overrun
        G_d2_driving 
        max_braking_torque
    end
    
    methods
        function obj = Powertrain(redline,shift_point,gears,torque_fn,final_drive,wheel_radius,...
                drivetrain_efficiency,G_d1,G_d2_overrun,G_d2_driving,brake_distribution,max_braking_torque)
            obj.redline = redline;
            obj.shift_point = shift_point;
            obj.gears = gears;
            obj.torque_fn = torque_fn;
            obj.final_drive = final_drive;
            obj.wheel_radius = wheel_radius;  
            obj.drivetrain_efficiency = drivetrain_efficiency;
            obj.G_d1 = G_d1;
            obj.G_d2_overrun = G_d2_overrun;
            obj.G_d2_driving = G_d2_driving;
            obj.brake_distribution = brake_distribution;            
            obj.max_braking_torque = max_braking_torque;
        end
                
        function out = drivetrain_reduction (obj,current_gear)
            % input current_gear means gear number
            % output drivetrain_reduction means total gearing from engine to axle
            out = obj.gears(current_gear)*obj.final_drive;
        end
        
        function [engine_rpm,current_gear] = engine_rpm(obj,omega_3,omega_4,long_vel)
            % calculates longitudinal velocities to switch gears at
            % approximately equal to redline but a tiny bit off due to wheel slips
            switch_gear_velocities = obj.shift_point./obj.gears/obj.final_drive*pi/30*obj.wheel_radius;
            % in final gear car can reach redline
            switch_gear_velocities(end) = obj.redline./obj.gears(end)/obj.final_drive*pi/30*obj.wheel_radius;
                        
            % finds lowest possible gear for given longitudinal velocity
            current_gear = find(long_vel<switch_gear_velocities,1);
            engine_rpm = (omega_3+omega_4)/2*obj.drivetrain_reduction(current_gear)*30/pi; %rpm      
            %engine_rpm = (omega_3+omega_4)/2*obj.final_drive(current_gear)*30/pi; %rpm          
        end            

        function [T_1,T_2,T_3,T_4] = wheel_torques(obj, engine_rpm, omega_3, omega_4, throttle, current_gear)
            % outputs wheel torques
            % driving torque is positive, braking is negative (opposite of SAE convention)
            
            if throttle > 0 % accelerating
                % linear interpolation of torque curve
                % linterp1 is faster than MATLAB interp1q, credit Jeffrey Wu
                torque_engine = throttle*lininterp1(obj.torque_fn(1,:),obj.torque_fn(2,:),engine_rpm); 
                
                torque_engine = torque_engine*1.35581795; %ft-lb to nm
                
                torque_engine = torque_engine*obj.drivetrain_efficiency; 

                % differential model
                torque_drive = torque_engine*obj.drivetrain_reduction(current_gear);

                if torque_drive < 0 % overrun
                    torque_transfer = -obj.G_d1-obj.G_d2_overrun*torque_drive;
                elseif torque_drive >= 0 % driving
                    torque_transfer = -obj.G_d1+obj.G_d2_driving*torque_drive;
                    %(obj.G_d1+obj.G_d2_driving*torque_drive
                end

                delta_t = torque_transfer*sign(omega_4-omega_3); % torque transfer
                
                T_1 = 0;
                T_2 = 0;
                T_3 = (torque_engine*obj.drivetrain_reduction(current_gear))/2+delta_t;
                T_4 = (torque_engine*obj.drivetrain_reduction(current_gear))/2-delta_t;               
            
            elseif throttle <= 0 % braking
                torque_braking = throttle*obj.max_braking_torque;
                
                torque_engine = 0; % engine drag torque with 0 throttle
                delta_t = 0; % differential has no effect when braking
                
                T_1 = (torque_braking*obj.brake_distribution)/2;
                T_2 = (torque_braking*obj.brake_distribution)/2;
                T_3 = (torque_braking*(1-obj.brake_distribution))/2+...
                    (torque_engine*obj.drivetrain_reduction(current_gear))/2+delta_t;
                T_4 = (torque_braking*(1-obj.brake_distribution))/2+...
                    (torque_engine*obj.drivetrain_reduction(current_gear))/2-delta_t;
            end
        end
    end
    
end

