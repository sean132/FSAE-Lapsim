classdef Events
    % Calculates results for different dynamic events
    
    properties
        car
        accel_car
        vel_matrix_accel
        vel_matrix_braking
        
        autocross_track
        endurance_track
        
        % used for interpolation in Autocross
        x_table_corner_vel
        radius_vector
        max_vel_corner_vector
        
        % used for interpolation in Accel
        x_table_accel
        long_vel_guess
        long_accel_matrix
    end
    
    methods
        function obj = Events(car,accel_car,vel_matrix_accel,vel_matrix_braking)
            obj.car = car;
            obj.accel_car = accel_car;
            obj.vel_matrix_accel = vel_matrix_accel;
            obj.vel_matrix_braking = vel_matrix_braking;
            
            % maps
            load('track_autocross_2017.mat');
            obj.autocross_track = [arclength; curvature];
            load('track_endurance_2017.mat');
            obj.endurance_track = [arclength; curvature];
            
            % sweep for max velocity for given radius
            % used for interpolation in Autocross
            [x_table_corner_vel,radius_vector,max_vel_corner_vector] = vel_cornering_sweep(car);
            obj.x_table_corner_vel = x_table_corner_vel;
            obj.radius_vector = radius_vector;
            obj.max_vel_corner_vector = max_vel_corner_vector;
            
            % sweep for max pure longitudinal acceleration for given velocity
            % used for interpolation in Accel
            [x_table_accel,long_vel_guess,long_accel_matrix] = long_accel_sweep(accel_car);
            obj.x_table_accel = x_table_accel;
            obj.long_vel_guess = long_vel_guess;
            obj.long_accel_matrix = long_accel_matrix;
                        
        end
        
        function [x_table_skid,max_vel_skid,skidpad_time] = Skidpad(obj)
            % modelled as pure steady state (no longitudinal acceleration)
            % inner radius of skidpad is 7.625 m
            % width of skidpad is 3 m
            % old lapsim used 8.55 radius - essentially means ~ 1 ft gap from cones

            radius = 8.5;
            [x_table_skid,max_vel_skid,skidpad_time] = max_skidpad_vel(radius,obj.car);
        end
        
        function [time,ending_vel,long_accel_vector,long_vel_vector] = Accel(obj)
            % car starts 0.3 m behind starting line
            % accel is 75 m long
            
            long_vel = 0;
                      
            long_vel_interp = obj.long_vel_guess;
            long_accel_interp = obj.long_accel_matrix;
            
            [~,ending_vel,~,~] = straight(long_vel,0.3,long_vel_interp,...
                long_accel_interp,obj.accel_car.max_vel);
            
            % starting velocity for accel is ending velocity of 0.3 straight
            long_vel = ending_vel;
            
            [time,ending_vel,long_accel_vector,long_vel_vector] = straight(long_vel,75,...
                long_vel_interp,long_accel_interp,obj.accel_car.max_vel);
            
        end
        
        function [long_vel_final,long_accel_final,lat_accel_final,time_final] = Track_Solver(obj,arclength,curvature)
            % finds apexes in curvature profile (apex of corner) and finds
            %   max possible velocity at each apex
            % then max accel and max braking are calculated for each
            %   segment between apexes
            % the minimum of the profiles is used to calculate velocity and
            %   acceleration profiles as well as time
            % for more theoretical detail of method see Siegler p. 20 or Brayshaw p. 52

            % find apexes in curvature profile (apex of corner has smallest turn radius)
            [extrema,extrema_indices] = curvature_apexes(arclength,curvature);

            arclength = [0 arclength];

            % find max possible velocity at each apex
            [apex_velocity] = apex_velocities(obj.radius_vector,obj.max_vel_corner_vector,extrema);

            % F_accel/braking(lat_accel,long_vel) returns the max possible accel/braking
            [F_accel,F_braking] = create_scattered_interpolants(obj.vel_matrix_accel,...
                obj.vel_matrix_braking);

            % Maximum possible acceleration between apexes
            % calculating velocity and acceleration profiles as well as time

            % car starts 6 m behind starting line 
            long_vel_interp = obj.long_vel_guess;
            long_accel_interp = obj.long_accel_matrix;
            [~,ending_vel,~,~] = straight(0,6,long_vel_interp,long_accel_interp,obj.car.max_vel);

            % starting velocity is ending velocity of straight
            long_vel = ending_vel; 

            lat_accel_vector_1 = [];
            long_accel_vector_1 = [];
            long_vel_vector_1 = [];
            time_1 = [];

            lat_accel_vector_2 = [];
            long_accel_vector_2 = [];
            long_vel_vector_2 = [];
            time_2 = [];

            last_index_1 = 1;
            last_index_2 = 1;

            % calculates the maximum possible acceleration and braking between each
            %   apex pair

            for j = 1:numel(extrema_indices) % loop through each segment
                % find max acceleration from initial velocity to next apex
                for i = last_index_1:extrema_indices(j) % segment from previous apex to next        
                    % basic kinematics equations
                    lat_accel = long_vel^2*abs(curvature(i))/9.81;
                    lat_accel_vector_1(i) = lat_accel*sign(curvature(i));
                    long_accel = F_accel(lat_accel,long_vel)*9.81;
                    if long_vel == obj.car.max_vel
                        long_accel = 0;
                    end
                    long_accel_vector_1(i) = long_accel;
                    long_vel_initial = long_vel;
                    long_vel_vector_1(i) = long_vel_initial;
                    long_vel = sqrt(long_vel^2+2*long_accel*(arclength(i+1)-arclength(i)));
                    % can't exceed max possible velocity for given radius
                    long_vel = min(long_vel,lininterp1(obj.radius_vector,obj.max_vel_corner_vector,...
                        abs(1/curvature(i)))); 
                    time_1(i) = 2*(arclength(i+1)-arclength(i))/(long_vel+long_vel_initial);
                end

                % start next segment from end of current segment
                last_index_1 = extrema_indices(j);

                % for braking calculate backwards from the apex velocity of the segment end
                long_vel = apex_velocity(j);

                for i = extrema_indices(j):-1:last_index_2 % opposite direction from accel

                    % basic kinematics equations
                    lat_accel = long_vel^2*abs(curvature(i))/9.81;
                    lat_accel_vector_2(i) = lat_accel*sign(curvature(i));
                    long_accel = F_braking(lat_accel,long_vel)*9.81;
                    if long_vel == obj.car.max_vel
                        long_accel = 0;
                    end
                    long_accel_vector_2(i) = long_accel;
                    long_vel_initial = long_vel;
                    long_vel_vector_2(i) = long_vel_initial;
                    long_vel = sqrt(long_vel^2-2*long_accel*(arclength(i+1)-arclength(i)));
                    % can't exceed max possible velocity for given radius
                    long_vel = min(long_vel,lininterp1(obj.radius_vector,obj.max_vel_corner_vector,...
                        abs(1/curvature(i)))); 
                    time_2(i) = 2*(arclength(i+1)-arclength(i))/(long_vel+long_vel_initial);
                end

                % end next calculation at end of current segment
                last_index_2 = extrema_indices(j);

                % if apex velocity can not be reached, e.g. segment is too short to
                %   reach apex velocity, then the ending velocity is the velocity
                %   reached during acceleration
                long_vel = min(long_vel_vector_1(end),long_vel_vector_2(end));

            end  
            
            % final longitudinal velocity is minimum of acceleration and braking
            %   velocity profiles
            long_vel_final = min(long_vel_vector_1,long_vel_vector_2);
            indices_2 = find(long_vel_vector_2 == long_vel_final);

            % replace acceleration and time with correct profile
            long_accel_final = long_accel_vector_1;
            long_accel_final(indices_2) = long_accel_vector_2(indices_2);
            long_accel_final = long_accel_final/9.81;
            lat_accel_final = lat_accel_vector_1;
            lat_accel_final(indices_2) = lat_accel_vector_2(indices_2);
            time_final = time_1;
            time_final(indices_2) = time_2(indices_2);
            % final time result
            time_final = sum(time_final); 
        end
        
        function [long_vel_final,long_accel_final,lat_accel_final,time_final] = Autocross(obj)
            arclength = obj.autocross_track(1,:);
            curvature = obj.autocross_track(2,:);
            [long_vel_final,long_accel_final,lat_accel_final,time_final] = ...
                Track_Solver(obj,arclength,curvature);            
        end
        
        function [long_vel_final,long_accel_final,lat_accel_final,time_final] = Endurance(obj)
            arclength = obj.endurance_track(1,:);
            curvature = obj.endurance_track(2,:);
            [long_vel_final,long_accel_final,lat_accel_final,time_final] = ...
                Track_Solver(obj,arclength,curvature);
            time_final = time_final*16; % 16 laps in endurance
        end
    end
end

