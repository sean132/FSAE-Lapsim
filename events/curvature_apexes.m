function [extrema,extrema_indices] = curvature_apexes(arclength,curvature)
% output extrema: curvature value at apexes
% output extrema_indices: indices of extrema

% finds maxima and minima in curvature profile (apexes of corner)
[maxima,maxima_indices] = findpeaks(curvature);
[minima,minima_indices] = findpeaks(-curvature);
extrema_matrix = [maxima minima; maxima_indices minima_indices]';

% sort rows based on second column so that indices are increasing
sorted_extrema_matrix = sortrows(extrema_matrix,2);  
extrema = sorted_extrema_matrix(:,1)';
extrema_indices = sorted_extrema_matrix(:,2)';

% add last point to apexes
extrema = [extrema curvature(end)];
extrema_indices = [extrema_indices numel(curvature)];

% visual check if necessary
% scatter(arclength(maxima_indices),curvature(maxima_indices));
% hold on
% scatter(arclength(minima_indices),curvature(minima_indices));
% plot(arclength,curvature);
