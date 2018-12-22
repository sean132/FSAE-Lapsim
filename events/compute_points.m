function [skidpad_points,accel_points,autocross_points,endurance_points] = compute_points(skidpad_time,...
    accel_time,autocross_time,endurance_time)

% computes dynamic event points
% based on 2018/2019 rules

% winning times (based on 2017 Lincoln)
skidpad_winning_time = 4.868;
accel_winning_time = 4.186;
autocross_winning_time = 51.516;
endurance_winning_time = 1272.016;

% skidpad
t_your = skidpad_time;
t_min = min([skidpad_winning_time t_your]);
t_max = 1.25 * t_min;
if t_your > t_max
   skidpad_points = 3.5;
else
   skidpad_points =  71.5 * ((t_max / t_your)^2.0 - 1.0) / ((t_max / t_min)^2.0 - 1.0) + 3.5;
end


% accel
t_your = accel_time;
t_min = min(accel_winning_time, t_your);
t_max = t_min * 1.5;
if t_your > t_max
    accel_points = 4.5;
else
    accel_points = 95.5 * ((t_max / t_your) - 1.0) / ((t_max / t_min) - 1.0) + 4.5;
end

% autocross
t_your = autocross_time;
t_min = min([autocross_winning_time, t_your]);
t_max = 1.45 * t_min;
if t_your > t_max
    autocross_points = 6.5;
else
    autocross_points = 118.5 * ((t_max / t_your) - 1.0) / ((t_max / t_min) - 1.0) + 6.5;
end

% endurance
t_your = endurance_time;
t_min = min(endurance_winning_time, t_your);
t_max = 1.45 * t_min;
if t_your > t_max
    endurance_points = 25;
else 
    endurance_points = 200 * ((t_max / t_your) - 1.0) / ((t_max / t_min) - 1.0) + 25.0;
end

end

