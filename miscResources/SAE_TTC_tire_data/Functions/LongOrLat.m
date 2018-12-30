function mode = LongOrLat()
%mode = LongOrLat()
%=========================
%Data selection
%=========================
%This function is used to determine whether the user wants to process
%lateral or longitudinal data. 
%===============
%OUTPUT ARGUMENTS
%===============
%mode = type of data
%===============
answer = questdlg('Which type of data would you like to process?',...
    'Data type', 'Longitudinal','Lateral','Longitudinal');

switch answer
    case 'Longitudinal'
        mode = 1;
    case 'Lateral'
        mode = 2;
end

