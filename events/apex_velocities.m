function [apex_velocity] = apex_velocities(radius,max_vel_corner_vector,extrema)
% find maximum possible velocity at each apex using interpolation

% radius at apexes is 1/curvature
extrema_radii = 1./extrema;

apex_velocity = zeros(size(extrema_radii));

for i = 1:numel(extrema_radii)
    apex_velocity(i) = lininterp1(radius,max_vel_corner_vector,abs(extrema_radii(i)));
end