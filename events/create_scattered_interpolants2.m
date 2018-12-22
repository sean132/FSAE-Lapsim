function [F_accel,F_braking] = create_scattered_interpolants2(vel_matrix_accel,vel_matrix_braking)
% input: vel_matrix_accel and vel_matrix_braking obtained from g-g diagram, 
%   contains velocity, lateral acceleration, and longitudinal acceleration
% output: scattered interpolants for accel and braking
%   finds max possible accel/braking given a lateral accel and velocity

long_g_accel = vel_matrix_accel(:,1);
lat_g_accel = vel_matrix_accel(:,2);
vel_accel = vel_matrix_accel(:,3);

long_g_braking = -vel_matrix_braking(:,1);
lat_g_braking = vel_matrix_braking(:,2);
vel_braking = vel_matrix_braking(:,3);

x = lat_g_accel;
y = vel_accel;
z = long_g_accel;

F_accel = scatteredInterpolant([x y],z);

x = lat_g_braking;
y = vel_braking;
z = long_g_braking;

F_braking = scatteredInterpolant([x y],z);

end

