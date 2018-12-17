function [x_table_corner_vel,radius,max_vel_corner_vector] = vel_cornering_sweep(car)
% run a sweep of velocity vs radius to be used for interpolation
% find max velocity the car can corner at for smallest radius

radius = 4.5;
[x_corner_vel,max_vel_corner,vel_corner_guess] = max_vel_cornering(radius,car.max_vel,car);
x_matrix = x_corner_vel;

% iterate through increasing radii
radius = 4.5:0.15:50;
max_vel_corner_vector = zeros(size(radius));
for i = 1:numel(radius)
    x0 = vel_corner_guess; % use previous solution as initial guess
    [x_corner_vel,max_vel_corner,vel_corner_guess] = max_vel_cornering(radius(i),car.max_vel,car,x0);
    max_vel_corner_vector(i) = max_vel_corner;
    x_matrix = [x_matrix;x_corner_vel];
end

x_table_corner_vel = generate_table(x_matrix);

% visual check if necessary
plot(radius,max_vel_corner_vector);

end

