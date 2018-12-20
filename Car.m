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
        
        ggPoints %g-g diagram points for car instance
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
        
        function [engine_rpm,beta,lat_accel,long_accel,yaw_accel,wheel_accel,omega,current_gear,...
                Fzvirtual,Fz,alpha,T] = equations(obj,P)           
            
            % inputs: vehicle parameters
            % outputs: vehicle accelerations and other properties
            
            % state and control matrix
            steer_angle = P(1); % deg
            throttle = P(2); % -1 for full braking, 1 for full throttle
            long_vel = P(3); % m/s
            lat_vel = P(4); % m/s
            yaw_rate = P(5); % equal to long_vel/radius (v/r)            
            kappa = P(6:9);
            % note: 1 = front left tire, 2 = front right tire
            %       3 = rear left tire, 4 = rear right tire
            
            % Powertrain
            omega = zeros(1,4);
            omega(1) = (kappa(1)+1)/obj.R*(long_vel+yaw_rate*obj.t_f/2);
            omega(2) = (kappa(2)+1)/obj.R*(long_vel-yaw_rate*obj.t_f/2);
            omega(3) = (kappa(3)+1)/obj.R*(long_vel+yaw_rate*obj.t_f/2);
            omega(4) = (kappa(4)+1)/obj.R*(long_vel-yaw_rate*obj.t_f/2);
                        
            [engine_rpm,current_gear] = obj.powertrain.engine_rpm(omega(3),omega(4),long_vel);
            [T_1,T_2,T_3,T_4] = obj.powertrain.wheel_torques(engine_rpm, omega(3), omega(4), throttle, current_gear);
            T = [T_1,T_2,T_3,T_4];
            % Normal Loads
            Fz_front_static = (obj.M*9.81*obj.l_r+obj.aero.lift(long_vel)*obj.aero.D_f)/obj.W_b;
            Fz_rear_static = (obj.M*9.81*obj.l_f+obj.aero.lift(long_vel)*obj.aero.D_r)/obj.W_b;
            
            long_load_transfer = (sum(T))/obj.R*(obj.h_g/obj.W_b); %(F_x1+F_x2+F_x3+F_x4)*h_g/W_b approximated (neglecting wheel dynamics) since longitudinal forces are unknown

            lat_load_transfer_front = (yaw_rate*long_vel*obj.M)/obj.t_f*((obj.l_r*obj.h_rf)/obj.W_b+...
                obj.R_sf*(obj.h_g-obj.h_rc));
            lat_load_transfer_rear = (yaw_rate*long_vel*obj.M)/obj.t_r*((obj.l_r*obj.h_rr)/obj.W_b+...
                (1-obj.R_sf)*(obj.h_g-obj.h_rc));
            
            % wheel load constraint method from Kelly
            Fzvirtual = zeros(1,4);
            Fzvirtual(1) = 0.5*Fz_front_static-0.5*long_load_transfer+lat_load_transfer_front;
            Fzvirtual(2) = 0.5*Fz_front_static-0.5*long_load_transfer-lat_load_transfer_front;
            Fzvirtual(3) = 0.5*Fz_rear_static+0.5*long_load_transfer+lat_load_transfer_rear;
            Fzvirtual(4) = 0.5*Fz_rear_static+0.5*long_load_transfer-lat_load_transfer_rear;

            % smooth approximation of max function
            epsilon = 10;
            Fz = (Fzvirtual + sqrt(Fzvirtual.^2 + epsilon))./2;

            % Tire Slips
            beta = atan(lat_vel/long_vel)*180/pi; % vehicle slip angle in deg
            
            steer_angle_1 = steer_angle; % could be modified for ackermann steering 
            steer_angle_2 = steer_angle;
            
            % slip angles (small angle assumption)
            alpha(1) = -steer_angle_1+(lat_vel+obj.l_f*yaw_rate)/(long_vel+yaw_rate*obj.t_f/2)*180/pi; %deg
            alpha(2) = -steer_angle_2+(lat_vel+obj.l_f*yaw_rate)/(long_vel-yaw_rate*obj.t_f/2)*180/pi; %deg
            alpha(3) = (lat_vel-obj.l_r*yaw_rate)/(long_vel+yaw_rate*obj.t_r/2)*180/pi;
            alpha(4) = (lat_vel-obj.l_r*yaw_rate)/(long_vel-yaw_rate*obj.t_r/2)*180/pi;
         
            % Tire Forces
            steer_angle = steer_angle_1*pi/180; % deg to rad
            [Fx,Fy,Fxw] = obj.tireForce(steer_angle,alpha,kappa,Fz);
            
            % Equations of Motion
            lat_accel = sum(Fy)*(1/obj.M)-yaw_rate*long_vel;
            long_accel = (sum(Fx)-obj.aero.drag(long_vel))*(1/obj.M)+yaw_rate*lat_vel;
            yaw_accel = ((Fx(1)-Fx(2))*obj.t_f/2+(Fx(3)-Fx(4))*obj.t_r/2+(Fy(1)+Fy(2))*obj.l_f-(Fy(3)+Fy(4))*obj.l_r)*(1/obj.I_zz);
            
            % neglects wheel rotational dynamics: for justification see Koutrik p.16
            wheel_accel(1) = (T(1)-Fxw(1)*obj.R);
            wheel_accel(2) = (T(2)-Fxw(2)*obj.R);
            wheel_accel(3) = (T(3)-Fx(3)*obj.R);
            wheel_accel(4) = (T(4)-Fx(4)*obj.R); 
        end
        function [Fx,Fy, F_xw] = tireForce(obj,steer_angle,alpha,kappa,Fz)
            %radians
            
            % forces in tire frame of reference
            F_xw1 = obj.tire.F_x(alpha(1),kappa(1),Fz(1)); 
            F_yw1 = obj.tire.F_y(alpha(1),kappa(1),Fz(1));
            F_xw2 = obj.tire.F_x(alpha(2),kappa(2),Fz(2));
            F_yw2 = obj.tire.F_y(alpha(2),kappa(2),Fz(2));
            F_xw = [F_xw1; F_xw2];

            % forces in vehicle frame of reference
            F_x1 = F_xw1*cos(steer_angle)-F_yw1*sin(steer_angle);
            F_y1 = F_xw1*sin(steer_angle)+F_yw1*cos(steer_angle);
            F_x2 = F_xw2*cos(steer_angle)-F_yw2*sin(steer_angle);
            F_y2 = F_xw2*sin(steer_angle)+F_yw2*cos(steer_angle);
            
            F_x3 = obj.tire.F_x(alpha(3),kappa(3),Fz(3));
            F_y3 = obj.tire.F_y(alpha(3),kappa(3),Fz(3));
            F_x4 = obj.tire.F_x(alpha(4),kappa(4),Fz(4));
            F_y4 = obj.tire.F_y(alpha(4),kappa(4),Fz(4));
            Fx = [F_x1; F_x2; F_x3; F_x4];
            Fy = [F_y1; F_y2; F_y3; F_y4];
        end
        function xdot = dynamics(obj,x)
%             1: yaw angle
%             2: yaw rate
%             3: long velocity
%             4: lat velocity
%             5: Xcg
%             6: Ycg
%             7: FL angular position
%             8: FL angular velocity
%             9: FR angular position
%             10: FR angular velocity
%             11: RL angular position
%             12: RL angular velocity
%             13: RR angular position
%             14: RR angular velocity
%             lf: cg to front, lr: cg to rear
%             u(1): steering input

              

%             xdot(1) = x(2);
%             xdot(2) = ((Fx1-Fx2)*(obj.t_f/2) + (Fx3-Fx4)*(obj.t_r/2) + (Fy1+Fy2)*obj.l_f - (Fy3+Fy4)*obj.l_r)/obj.I_zz;
%             xdot(3) = (Fx1+Fx2+Fx3+Fx4-Fax)/obj.M + x(2)*x(4);
%             xdot(4) = (Fy1+Fy2+Fy3+Fy4)/obj.M - x(2)*x(3);
%             xdot(5) = x(3)*cos(x(1))-x(4)*sin(x(1));
%             xdot(6) = x(3)*sin(x(1))+x(4)*cos(x(1));
%             xdot(7) = x(8);
%             xdot(8) = (T1 - Fxwl*Rf)/Jwf;
%             xdot(11) = x(12);
%             xdot(12) = ((T3-Fx3*Rr)*(Jwr+Jm*(Gr/2)^2)-(T4-Fx4*Rr)*Jm*(Gr/2)^2)/(Jwr^2+2*Jwr*Jm*(Gr/2)^2);
%             xdot(13) = x(14);
%             xdot(14) = ((T4-Fx4*Rr)*(Jwr+Jm(Gr/2)^2)-(T3-Fx3*Rr)*Jm*(Gr/2)^2)/(Jwr^2+2*Jwr*Jm*(Gr/2)^2);
            
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
        
        function [c,ceq] = constraint1(obj,P)            
            % no longitudinal acceleration constraint
            % used for optimizing longitudinal acceleration/braking           
            P(9) = P(8);
            
            [engine_rpm,beta,lat_accel,long_accel,yaw_accel,wheel_accel,omega,current_gear,...
                Fzvirtual,Fz,alpha,T] = obj.equations(P);
            c = [engine_rpm-13000,abs(beta)-20,-Fzvirtual(1:4)];
            ceq = [lat_accel,yaw_accel,wheel_accel(1:4)];
        end
        
        function [c,ceq] = constraint2(obj,P,long_accel_value)
            % longitudinal acceleration constrained to equal long_accel_value
            % used for optimizing lateral force for given longitudinal acceleration
            
            [engine_rpm,beta,lat_accel,long_accel,yaw_accel,wheel_accel,omega,current_gear,...
                Fzvirtual,Fz,alpha,T]...
                = obj.equations(P);
            c = [engine_rpm-13000,abs(beta)-20,-Fzvirtual(1:4)];
            ceq = [lat_accel,long_accel-long_accel_value,yaw_accel,wheel_accel(1:4)];
        end
        
        function [c,ceq] = constraint3(obj,P,radius)
            % longitudinal acceleration constrained to equal zero
            % velocity divided by yaw rate constrained to equal inputted radius
            % used for solving skidpad (optimizing velocity for zero longitudinal acceleration
            
            [engine_rpm,beta,lat_accel,long_accel,yaw_accel,wheel_accel,omega,current_gear,...
                Fzvirtual,Fz,alpha,T] = obj.equations(P);
            c = [engine_rpm-13000,abs(beta)-20,-Fzvirtual(1:4)];
            ceq = [P(3)/(P(5))-radius,lat_accel,long_accel,yaw_accel,wheel_accel(1:4)];
        end
        
        function [c,ceq] = constraint4(obj,P,lat_accel_value) 
            % lateral acceleration constrained to equal lat_accel_value
            % used for optimizing longitudinal acceleration for given lateral acceleration
            
            [engine_rpm,beta,lat_accel,long_accel,yaw_accel,wheel_accel,omega,current_gear,...
                Fzvirtual,Fz,alpha,T]...
                = obj.equations(P);
            c = [engine_rpm-13000,abs(beta)-20,-Fzvirtual(1:2)];
            ceq = [P(3)*P(5)-lat_accel_value,lat_accel,yaw_accel,wheel_accel(1:4)];
        end
        
        function [c,ceq] = constraint5(obj,P,radius)  
            % velocity divided by yaw rate constrained to equal inputted radius
            % used for calculating max velocity the car can corner at for given radius
            
            [engine_rpm,beta,lat_accel,long_accel,yaw_accel,wheel_accel,omega,current_gear,...
                Fzvirtual,Fz,alpha,T]...
                = obj.equations(P);
            c = [engine_rpm-13000,abs(beta)-20,-Fzvirtual(1:4)];
            ceq = [P(3)/(P(5))-radius,lat_accel,yaw_accel,wheel_accel(1:4)];
        end
        
        function [c,ceq] = constraint6(obj,P)            
            % no longitudinal acceleration constraint
            % used for optimizing lateral acceleration            
            
           [engine_rpm,beta,lat_accel,long_accel,yaw_accel,wheel_accel,omega,current_gear,...
                Fzvirtual,Fz,alpha,T]...
                = obj.equations(P);
            c = [engine_rpm-13000,abs(beta)-20,-Fzvirtual(1:4)];
            ceq = [lat_accel,yaw_accel,wheel_accel(1:4)];
        end
        
        function [c,ceq] = constraint7(obj,P)
            % longitudinal acceleration constrained to 0
            % no yaw accel constraint
            % used to determine terminal under/oversteer            
            
            [engine_rpm,beta,lat_accel,long_accel,yaw_accel,wheel_accel,omega,current_gear,...
                Fzvirtual,Fz,alpha,T]...
                = obj.equations(P);
            c = [engine_rpm-13000,abs(beta)-20,-Fzvirtual(1:4)];
            ceq = [lat_accel,long_accel,wheel_accel(1:4)];
        end
        
        function [c,ceq] = constraint8(obj,P,radius,long_vel_value)  
            % velocity divided by yaw rate constrained to equal inputted radius
            % velocity constrained to equal long_vel_value
            % longitudinal acceleration constrained to 0
            % used for constant radius test
                        
           [engine_rpm,beta,lat_accel,long_accel,yaw_accel,wheel_accel,omega,current_gear,...
                Fzvirtual,Fz,alpha,T]...
                = obj.equations(P);
            c = [engine_rpm-13000,abs(beta)-20,-Fzvirtual(1:4)];
            ceq = [P(3)/(P(5))-radius,P(3)-long_vel_value,lat_accel,...
                long_accel, yaw_accel,wheel_accel(1:4)];
        end
        
        % objective function
        function out = long_accel(obj,P)
            % used for optimizing longitudinal acceleration            
            P(9) = P(8);
            
            [engine_rpm,beta,lat_accel,long_accel,yaw_accel,wheel_accel,omega,current_gear,...
                Fzvirtual,Fz,alpha,T]...
                = obj.equations(P);
            out = long_accel;
        end
        
        % maximum possible car velocity
        function out = max_vel(obj)
            out = obj.powertrain.redline*pi/30*obj.R/...
                obj.powertrain.drivetrain_reduction(numel(obj.powertrain.gears))-0.001;
        end
    end
    
end

