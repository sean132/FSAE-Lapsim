classdef Car
    % 3 DOF Model
    % equations adapted from Casanova Appendix
    
    properties
        M
        W_b
        l_f
        l_r
        t_f
        t_r
        h_rr
        h_rf
        h_rc
        R
        h_g
        R_sf
        I_zz
        aero
        powertrain
        tire        
    end
    
    methods
        function obj = Car(mass,wheelbase,weight_dist,track_width,wheel_radius,cg_height,...
                roll_center_height_front,roll_center_height_rear,R_sf,I_zz,aero,powertrain,tire)
            obj.M = mass;
            obj.W_b = wheelbase;
            obj.l_f = wheelbase*weight_dist; % distance from cg to front
            obj.l_r = wheelbase*(1-weight_dist); % distance from cg to rear
            obj.t_f = track_width;
            obj.t_r = track_width;
            obj.h_rr = roll_center_height_rear;
            obj.h_rf = roll_center_height_front;
            obj.h_rc = (obj.h_rf+obj.h_rr)/2; % approximation of roll center height at cg
            obj.R = wheel_radius;
            obj.h_g = cg_height;
            obj.R_sf = R_sf;
            obj.I_zz = I_zz;
            obj.aero = aero;
            obj.powertrain = powertrain;
            obj.tire = tire;
        end
        
        function [engine_rpm,beta,lat_accel,long_accel,yaw_accel,wheel_accel_1,wheel_accel_2,...
                wheel_accel_3,wheel_accel_4,omega_1,omega_2,omega_3,omega_4,current_gear,...
                Fz_1_virtual,Fz_2_virtual,Fz_3_virtual,Fz_4_virtual,Fz_1,Fz_2,Fz_3,Fz_4,...
                alpha_1,alpha_2,alpha_3,alpha_4,T_1,T_2,T_3,T_4] = equations(obj,P,scaling_factor)           
            
            P = P.*scaling_factor;
            
            % inputs: vehicle parameters
            % outputs: vehicle accelerations and other properties
            
            % state and control matrix
            steer_angle = P(1); % deg
            throttle = P(2); % -1 for full braking, 1 for full throttle
            long_vel = P(3); % m/s
            lat_vel = P(4); % m/s
            yaw_rate = P(5); % equal to long_vel/radius (v/r)
            kappa_1 = P(6);  % slip ratios 
            kappa_2 = P(7);
            kappa_3 = P(8);
            kappa_4 = P(9);            
            
            % note: 1 = front left tire, 2 = front right tire
            %       3 = rear left tire, 4 = rear right tire
            
            %% Powertrain
            omega_1 = (kappa_1+1)/obj.R*(long_vel+yaw_rate*obj.t_f/2);
            omega_2 = (kappa_2+1)/obj.R*(long_vel-yaw_rate*obj.t_f/2);
            omega_3 = (kappa_3+1)/obj.R*(long_vel+yaw_rate*obj.t_f/2);
            omega_4 = (kappa_4+1)/obj.R*(long_vel-yaw_rate*obj.t_f/2);
                        
            [engine_rpm,current_gear] = obj.powertrain.engine_rpm(omega_3,omega_4,long_vel);
            [T_1,T_2,T_3,T_4] = obj.powertrain.wheel_torques(engine_rpm, omega_3, omega_4, throttle, current_gear);
            
            %% Normal Loads
            Fz_front_static = (obj.M*9.81*obj.l_r+obj.aero.lift(long_vel)*obj.aero.D_f)/obj.W_b;
            Fz_rear_static = (obj.M*9.81*obj.l_f+obj.aero.lift(long_vel)*obj.aero.D_r)/obj.W_b;
            
            long_load_transfer = (T_1+T_2+T_3+T_4)/obj.R*(obj.h_g/obj.W_b); %(F_x1+F_x2+F_x3+F_x4)*h_g/W_b approximated (neglecting wheel dynamics) since longitudinal forces are unknown

            lat_load_transfer_front = (yaw_rate*long_vel*obj.M)/obj.t_f*((obj.l_r*obj.h_rf)/obj.W_b+...
                obj.R_sf*(obj.h_g-obj.h_rc));
            lat_load_transfer_rear = (yaw_rate*long_vel*obj.M)/obj.t_r*((obj.l_r*obj.h_rr)/obj.W_b+...
                (1-obj.R_sf)*(obj.h_g-obj.h_rc));
            
            % wheel load constraint method from Kelly
            Fz_1_virtual = 0.5*Fz_front_static-0.5*long_load_transfer+lat_load_transfer_front;
            Fz_2_virtual = 0.5*Fz_front_static-0.5*long_load_transfer-lat_load_transfer_front;
            Fz_3_virtual = 0.5*Fz_rear_static+0.5*long_load_transfer+lat_load_transfer_rear;
            Fz_4_virtual = 0.5*Fz_rear_static+0.5*long_load_transfer-lat_load_transfer_rear;

            % smooth approximation of max function
            epsilon = 10;
            Fz_1 = (Fz_1_virtual+sqrt(Fz_1_virtual^2+epsilon))/2;
            Fz_2 = (Fz_2_virtual+sqrt(Fz_2_virtual^2+epsilon))/2;
            Fz_3 = (Fz_3_virtual+sqrt(Fz_3_virtual^2+epsilon))/2;
            Fz_4 = (Fz_4_virtual+sqrt(Fz_4_virtual^2+epsilon))/2;

            %% Tire Slips
            beta = atan(lat_vel/long_vel)*180/pi; % vehicle slip angle in deg
            
            steer_angle_1 = steer_angle; % could be modified for ackermann steering 
            steer_angle_2 = steer_angle;
            
            % slip angles (small angle assumption)
            alpha_1 = -steer_angle_1+(lat_vel+obj.l_f*yaw_rate)/(long_vel+yaw_rate*obj.t_f/2)*180/pi; %deg
            alpha_2 = -steer_angle_2+(lat_vel+obj.l_f*yaw_rate)/(long_vel-yaw_rate*obj.t_f/2)*180/pi; %deg
            alpha_3 = (lat_vel-obj.l_r*yaw_rate)/(long_vel+yaw_rate*obj.t_r/2)*180/pi;
            alpha_4 = (lat_vel-obj.l_r*yaw_rate)/(long_vel-yaw_rate*obj.t_r/2)*180/pi;
         
            %% Tire Forces
            steer_angle_1 = steer_angle_1*pi/180; % deg to rad
            steer_angle_2 = steer_angle_2*pi/180; % deg to rad
            
            % forces in tire frame of reference
            F_xw1 = obj.tire.F_x(alpha_1,kappa_1,Fz_1); 
            F_yw1 = obj.tire.F_y(alpha_1,kappa_1,Fz_1);
            F_xw2 = obj.tire.F_x(alpha_2,kappa_2,Fz_2);
            F_yw2 = obj.tire.F_y(alpha_2,kappa_2,Fz_2);

            % forces in vehicle frame of reference
            F_x1 = F_xw1*cos(steer_angle_1)-F_yw1*sin(steer_angle_1);
            F_y1 = F_xw1*sin(steer_angle_1)+F_yw1*cos(steer_angle_1);
            F_x2 = F_xw2*cos(steer_angle_2)-F_yw2*sin(steer_angle_2);
            F_y2 = F_xw2*sin(steer_angle_2)+F_yw2*cos(steer_angle_2);
            
            F_x3 = obj.tire.F_x(alpha_3,kappa_3,Fz_3);
            F_y3 = obj.tire.F_y(alpha_3,kappa_3,Fz_3);
            F_x4 = obj.tire.F_x(alpha_4,kappa_4,Fz_4);
            F_y4 = obj.tire.F_y(alpha_4,kappa_4,Fz_4);
            
            %% Equations of Motion
            lat_accel = (F_y1+F_y2+F_y3+F_y4)*(1/obj.M)-yaw_rate*long_vel;
            long_accel = (F_x1+F_x2+F_x3+F_x4-obj.aero.drag(long_vel))*(1/obj.M)+yaw_rate*lat_vel;
            yaw_accel = ((F_x1-F_x2)*obj.t_f/2+(F_x3-F_x4)*obj.t_r/2+(F_y1+F_y2)*obj.l_f-(F_y3+F_y4)*obj.l_r)*(1/obj.I_zz);
            
            % neglects wheel rotational dynamics: for justification see Koutrik p.16
            wheel_accel_1 = (T_1-F_xw1*obj.R);
            wheel_accel_2 = (T_2-F_xw2*obj.R);
            wheel_accel_3 = (T_3-F_x3*obj.R);
            wheel_accel_4 = (T_4-F_x4*obj.R);            
            
        end
        
        % These functions are used to set constraints for fmincon
        % input P: state and control vector containing:
        %   steer angle,throttle position,longitudinal velocity,
        %   lateral velocity,yaw rate,wheel rotational speeds 

        % output c: limits vehicle slip angle to less than 20 degrees (stability purposes)
        %   also limits engine rpm to below 13000
        %   also limits wheel loads to positive values (no wheel lift)
        % output ceq: constrains certain accelerations to 0 to satisfy
        %   steady-state conditions
        
        function [c,ceq] = constraint1(obj,P,scaling_factor)            
            % no longitudinal acceleration constraint
            % used for optimizing longitudinal acceleration/braking           
            P(9) = P(8);
            
            [engine_rpm,beta,lat_accel,~,yaw_accel,wheel_accel_1,wheel_accel_2,...
                wheel_accel_3,wheel_accel_4,~,~,~,~,~,Fz_1_virtual,Fz_2_virtual,Fz_3_virtual,Fz_4_virtual]...
                = obj.equations(P,scaling_factor);
            c = [engine_rpm-13000,abs(beta)-20,-Fz_1_virtual,-Fz_2_virtual,-Fz_3_virtual,-Fz_4_virtual];
            ceq = [lat_accel,yaw_accel,wheel_accel_1,wheel_accel_2,wheel_accel_3,wheel_accel_4];
        end
        
        function [c,ceq] = constraint2(obj,P,long_accel_value,scaling_factor)
            % longitudinal acceleration constrained to equal long_accel_value
            % used for optimizing lateral force for given longitudinal acceleration
            
            [engine_rpm,beta,lat_accel,long_accel,yaw_accel,wheel_accel_1,...
                wheel_accel_2,wheel_accel_3,wheel_accel_4,~,~,~,~,~,Fz_1_virtual,Fz_2_virtual,Fz_3_virtual,...
                Fz_4_virtual] = obj.equations(P,scaling_factor);
            c = [engine_rpm-13000,abs(beta)-20,-Fz_1_virtual,-Fz_2_virtual,-Fz_3_virtual,-Fz_4_virtual];
            ceq = [lat_accel,long_accel-long_accel_value,yaw_accel,wheel_accel_1,...
                wheel_accel_2,wheel_accel_3,wheel_accel_4];
        end
        
        function [c,ceq] = constraint3(obj,P,radius,scaling_factor)
            % longitudinal acceleration constrained to equal zero
            % velocity divided by yaw rate constrained to equal inputted radius
            % used for solving skidpad (optimizing velocity for zero longitudinal acceleration
            
            [engine_rpm,beta,lat_accel,long_accel,yaw_accel,wheel_accel_1,wheel_accel_2,...
                wheel_accel_3,wheel_accel_4,~,~,~,~,~,Fz_1_virtual,Fz_2_virtual,Fz_3_virtual,Fz_4_virtual]...
                = obj.equations(P,scaling_factor);
            c = [engine_rpm-13000,abs(beta)-20,-Fz_1_virtual,-Fz_2_virtual,-Fz_3_virtual,-Fz_4_virtual];
            ceq = [P(3)*scaling_factor(3)/(P(5)*scaling_factor(5))-radius,lat_accel,long_accel,yaw_accel,wheel_accel_1,...
                wheel_accel_2,wheel_accel_3,wheel_accel_4];
        end
        
        function [c,ceq] = constraint4(obj,P,lat_accel_value,scaling_factor) 
            % lateral acceleration constrained to equal lat_accel_value
            % used for optimizing longitudinal acceleration for given lateral acceleration
            
            [engine_rpm,beta,lat_accel,~,yaw_accel,wheel_accel_1,wheel_accel_2,...
                wheel_accel_3,wheel_accel_4,~,~,~,~,~,Fz_1_virtual,Fz_2_virtual,Fz_3_virtual,Fz_4_virtual]...
                = obj.equations(P,scaling_factor);
            c = [engine_rpm-13000,abs(beta)-20,-Fz_1_virtual,-Fz_2_virtual,-Fz_3_virtual,-Fz_4_virtual];
            ceq = [P(3)*scaling_factor(3)*P(5)*scaling_factor(5)-lat_accel_value,lat_accel,yaw_accel,wheel_accel_1,...
                wheel_accel_2,wheel_accel_3,wheel_accel_4];
        end
        
        function [c,ceq] = constraint5(obj,P,radius,scaling_factor)  
            % velocity divided by yaw rate constrained to equal inputted radius
            % used for calculating max velocity the car can corner at for given radius
            
            [engine_rpm,beta,lat_accel,~,yaw_accel,wheel_accel_1,wheel_accel_2,...
                wheel_accel_3,wheel_accel_4,~,~,~,~,~,Fz_1_virtual,Fz_2_virtual,Fz_3_virtual,Fz_4_virtual]...
                = obj.equations(P,scaling_factor);
            c = [engine_rpm-13000,abs(beta)-20,-Fz_1_virtual,-Fz_2_virtual,-Fz_3_virtual,-Fz_4_virtual];
            ceq = [P(3)*scaling_factor(3)/(P(5)*scaling_factor(5))-radius,lat_accel,yaw_accel,wheel_accel_1,wheel_accel_2,...
                wheel_accel_3,wheel_accel_4];
        end
        
        function [c,ceq] = constraint6(obj,P,scaling_factor)            
            % no longitudinal acceleration constraint
            % used for optimizing lateral acceleration            
            
            [engine_rpm,beta,lat_accel,~,yaw_accel,wheel_accel_1,wheel_accel_2,...
                wheel_accel_3,wheel_accel_4,~,~,~,~,~,Fz_1_virtual,Fz_2_virtual,Fz_3_virtual,Fz_4_virtual]...
                = obj.equations(P,scaling_factor);
            c = [engine_rpm-13000,abs(beta)-20,-Fz_1_virtual,-Fz_2_virtual,-Fz_3_virtual,-Fz_4_virtual];
            ceq = [lat_accel,yaw_accel,wheel_accel_1,wheel_accel_2,wheel_accel_3,wheel_accel_4];
        end
        
        function [c,ceq] = constraint7(obj,P,scaling_factor)
            % longitudinal acceleration constrained to 0
            % no yaw accel constraint
            % used to determine terminal under/oversteer            
            
            [engine_rpm,beta,lat_accel,long_accel,~,wheel_accel_1,...
                wheel_accel_2,wheel_accel_3,wheel_accel_4,~,~,~,~,~,Fz_1_virtual,Fz_2_virtual,Fz_3_virtual,...
                Fz_4_virtual] = obj.equations(P,scaling_factor);
            c = [engine_rpm-13000,abs(beta)-20,-Fz_1_virtual,-Fz_2_virtual,-Fz_3_virtual,-Fz_4_virtual];
            ceq = [lat_accel,long_accel,wheel_accel_1,...
                wheel_accel_2,wheel_accel_3,wheel_accel_4];
        end
        
        function [c,ceq] = constraint8(obj,P,radius,long_vel_value,scaling_factor)  
            % velocity divided by yaw rate constrained to equal inputted radius
            % velocity constrained to equal long_vel_value
            % longitudinal acceleration constrained to 0
            % used for constant radius test
                        
            [engine_rpm,beta,lat_accel,long_accel,yaw_accel,wheel_accel_1,wheel_accel_2,...
                wheel_accel_3,wheel_accel_4,~,~,~,~,~,Fz_1_virtual,Fz_2_virtual,Fz_3_virtual,Fz_4_virtual]...
                = obj.equations(P,scaling_factor);
            c = [engine_rpm-13000,abs(beta)-20,-Fz_1_virtual,-Fz_2_virtual,-Fz_3_virtual,-Fz_4_virtual];
            ceq = [P(3)*scaling_factor(3)/(P(5)*scaling_factor(5))-radius,P(3)-long_vel_value,lat_accel,...
                long_accel, yaw_accel,wheel_accel_1,wheel_accel_2,wheel_accel_3,wheel_accel_4];
        end
        
        % objective function
        function out = long_accel(obj,P,scaling_factor)
            % used for optimizing longitudinal acceleration            
            P(9) = P(8);
            
            [~,~,~,long_accel,~,~,~,~,~] = obj.equations(P,scaling_factor);
            out = long_accel;
        end
        
        % maximum possible car velocity
        function out = max_vel(obj)
            out = obj.powertrain.redline*pi/30*obj.R/...
                obj.powertrain.drivetrain_reduction(numel(obj.powertrain.gears))-0.001;
        end
    end
    
end

