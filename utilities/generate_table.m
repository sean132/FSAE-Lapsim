function [varargout] = generate_table(varargin)
% generates tables containing data (useful for debugging)
% can accept multiple matrices to output multiple tables

for i = 1:nargout
    x_matrix = varargin{i};
    x_cell = num2cell(x_matrix);
    x_table = cell2table(x_cell);
    x_table.Properties.VariableNames = {'exitflag','long_accel','lat_accel','steer_angle',...
        'throttle','long_vel','lat_vel','yaw_rate','kappa_1','kappa_2','kappa_3','kappa_4',...
        'omega_1','omega_2','omega_3','omega_4','engine_rpm','current_gear','beta'...
        'Fz_1','Fz_2','Fz_3','Fz_4','alpha_1','alpha_2','alpha_3','alpha_4','T_1','T_2','T_3','T_4'};
    varargout{i} = x_table;
end