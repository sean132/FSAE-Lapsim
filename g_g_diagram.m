function [vel_matrix_accel,vel_matrix_braking,x_table_ss,x_table_accel,x_table_braking] = ...
    g_g_diagram(car)
% creates velocity-dependent g-g diagram 
% describes max lateral acceleration, max longitudinal acceleration for
%   certain velocity
% input car: Car object
% outputs vel_matrix_accel,vel_matrix_braking: used for interpolation in
%   Events 
longVgrid = 25;
latAgrid = 30;
vel_guesses = linspace(5,car.max_vel-0.5,longVgrid);
vel_matrix_accel = [];
vel_matrix_braking = [];
x_matrix_ss = [];
max_lat_accel_matrix = zeros(size(vel_guesses));
vel_counter = 1;

% sweeps through different velocities
for long_vel_guess = vel_guesses
    %% Max Lat Accel    
    if vel_counter == 1
        [x_ss,lat_accel,lat_long_accel,lat_accel_guess] = max_lat_accel(long_vel_guess,car);
    else
        x0 = lat_accel_guess;
        [x_ss,lat_accel,lat_long_accel,lat_accel_guess] = max_lat_accel(long_vel_guess,car,x0);
    end
    x_matrix_ss = [x_matrix_ss; x_ss];
    max_lat_accel_matrix(vel_counter) = lat_accel;
    counter = 1;
    x_matrix_accel = [];
    x_matrix_braking = [];

    lat_accel_matrix = linspace(0.1,lat_accel-0.1,latAgrid);

    % sweeps through different lateral accelerations (0 to maximum)
    for i = lat_accel_matrix
        %% Max longitudinal acceleration at given lateral acceleration
        lat_accel_value = i*9.81; % g's to m/s^2
        
%         if counter == 1
            [x_accel,long_accel,long_accel_guess] = max_long_accel_cornering(long_vel_guess,...
                lat_accel_value,car);
%         else
%             x0 = long_accel_guess;
%             [x_accel,long_accel,long_accel_guess] = max_long_accel_cornering(long_vel_guess,...
%                 lat_accel_value,car);
%         end
                   
        x_matrix_accel = [x_matrix_accel; x_accel];
        long_vel_matrix(counter) = long_vel_guess;
        long_accel_matrix(counter) = long_accel;
        %% Max Braking
        lat_accel_value = i*9.81;
        
%         if counter == 1
            [x_braking,long_decel,braking_decel_guess] = max_braking_decel_cornering(long_vel_guess,...
                lat_accel_value,car);
%             fprintf("%0.2f %0.2f %0.2f\n",[long_decel long_vel_guess lat_accel_value]); 
%         else
%             x0 = braking_decel_guess;
%             [x_braking,long_decel,braking_decel_guess] = max_braking_decel_cornering(long_vel_guess,...
%                 lat_accel_value,car);
%         end
%         fprintf("%0.2f %0.2f %0.2f\n",[long_vel_guess lat_accel long_accel]);
%         fprintf("%0.2f %0.2f %0.2f\n",[long_vel_guess lat_accel long_decel]);
        x_matrix_braking = [x_matrix_braking; x_braking];
        long_vel_matrix(counter) = long_vel_guess;
        braking_matrix(counter) = long_decel;
            
        counter = counter+1;
    end

    % removes points which didn't converge (exitflag not equal to 1)
    long_accel_matrix = long_accel_matrix(x_matrix_accel(:,1)==1);
    braking_matrix = braking_matrix(x_matrix_braking(:,1)==1);
    lat_accel_matrix_accel = lat_accel_matrix(x_matrix_accel(:,1)==1);
    lat_accel_matrix_braking = lat_accel_matrix(x_matrix_braking(:,1)==1);
    
    % generate table
    [table_accel,table_braking] = generate_table(x_matrix_accel,x_matrix_braking);
    x_table_accel{vel_counter} = table_accel;
    x_table_braking{vel_counter} = table_braking;
    
    % add max lat accel point
    lat_accel_matrix_accel = [lat_accel_matrix_accel lat_accel];
    lat_accel_matrix_braking = [lat_accel_matrix_braking lat_accel];    
    long_accel_matrix = [long_accel_matrix lat_long_accel/9.81];
    braking_matrix = [braking_matrix lat_long_accel/9.81];

    % creates two matrices including lateral acceleration, longitudinal
    %   acceleration/braking, and velocity
    matrix_accel = [lat_accel_matrix_accel; long_accel_matrix; long_vel_guess*ones(size(lat_accel_matrix_accel))];
    matrix_braking = [lat_accel_matrix_braking; braking_matrix; long_vel_guess*ones(size(lat_accel_matrix_braking))];    
    vel_matrix_accel = [vel_matrix_accel matrix_accel];
    vel_matrix_braking = [vel_matrix_braking matrix_braking];
    
    vel_counter = vel_counter+1;
end

% generate table
[x_table_ss] = generate_table(x_matrix_ss);
    