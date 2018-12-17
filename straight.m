function [time,ending_vel,long_accel_vector,long_vel_vector] = straight(long_vel,length,...
    long_vel_interp,long_accel_interp,max_vel)
% inputs: length of straight and starting velocity
% outputs: time and ending velocity 

distance = [0 linspace(0,length,10000)];
long_accel_vector = zeros(1,10000);
long_vel_vector = zeros(1,10000);

%{
shift_time = 0.1;
obj = car.powertrain;
switch_gear_velocities = obj.shift_point./obj.gears/obj.final_drive*pi/30*obj.wheel_radius;
switch_gear_velocities(end) = obj.redline./obj.gears(end)/obj.final_drive*pi/30*obj.wheel_radius;
current_gear_vel = switch_gear_velocities;
%}

time = zeros(1,10000);
for i = 1:10000
    %{
    if long_vel>current_gear_vel(1) && long_vel_vector(i-1)<current_gear_vel(1)
        while shift_time_cumulative == time(i)
            long_accel = -car.aero.drag(long_vel)/car.M;
            current_gear_vel = current_gear_vel(2:end);
        end
    end
    %}
    
    % basic kinematics equations
    long_accel = lininterp1(long_vel_interp,long_accel_interp,long_vel)*9.81;
    if long_vel == max_vel
        long_accel = 0;
    end
    long_accel_vector(i) = long_accel;
    long_vel_initial = long_vel;
    long_vel_vector(i) = long_vel_initial;
    long_vel = sqrt(long_vel^2+2*long_accel*(distance(i+1)-distance(i)));
    % limit top speed to max velocity
    long_vel = min(long_vel,max_vel);
    time(i) = 2*(distance(i+1)-distance(i))/(long_vel+long_vel_initial);
end      

time = sum(time);

% velocity at end of straight
ending_vel = long_vel;

end

