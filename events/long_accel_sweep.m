function [x_table_accel,long_vel_guess,long_accel_matrix] = long_accel_sweep(car)
% run a sweep of longitudinal accel vs velocity to be used for interpolation
% find max acceleration the car can generate for a given velocity

long_vel_guess = linspace(0.001,car.max_vel,100);
long_accel_matrix = zeros(size(long_vel_guess));

x_matrix = [];

for i = 1:numel(long_vel_guess)
    if i == 1
        [x_accel,long_accel,long_accel_guess] = max_long_accel(long_vel_guess(i),car);
    else
        x0 = long_accel_guess;
        [x_accel,long_accel,long_accel_guess] = max_long_accel(long_vel_guess(i),car,x0);
    end
    x_matrix = [x_matrix; x_accel];   
    long_accel_matrix(i) = long_accel;
end

long_vel_guess = long_vel_guess(1:end-1);
long_accel_matrix = long_accel_matrix(1:end-1);
[x_table_accel] = generate_table(x_matrix);

% visual check
plot(long_vel_guess,long_accel_matrix);

end

