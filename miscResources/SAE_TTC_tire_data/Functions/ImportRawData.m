% Copyright 2017 The MathWorks, Inc.

function [ AMBTMP, ET, FX, FY, FZ, IA, MX, MZ, N, NFX, NFY, P, RE, RL, RST, SA, SR, TSTC, TSTI, TSTO, V] = ImportRawData( fileid )
%ImportRawData This function imports raw tire data from a tiredata *.dat

[~ , ~ , ext ] = fileparts(fileid);

if ext=='.dat'
    lat_test = importdata(fileid);
    %% 
    % * AMBTMP: Ambient Temperature
    AMBTMP = lat_test.data(:,20);
    %% 
    % * ET: Elapsed Time [s]
    ET = lat_test.data(:,1); 
    %% 
    % * FX: Traction Force [N]
    FX = lat_test.data(:,9);
    %% 
    % * FY: Lateral Force [N]
    FY = lat_test.data(:,10);
    %% 
    % * FZ: Vertical Force [N]
    FZ = -lat_test.data(:,11);
    %% 
    % * IA: Inclination Angle [deg]
    IA = lat_test.data(:,5);
    %% 
    % * MX: Overturning Moment [Nm]
    MX = lat_test.data(:,12);
    %% 
    % * MZ: Aligning Moment [Nm]
    MZ = lat_test.data(:,13);
    %% 
    % * N: Wheel rotational speed [rpm]
    N = lat_test.data(:,3);
    %% 
    % * NFX: Normal Force in X [normalized against vectorial load]
    NFX = lat_test.data(:,14); 
    %% 
    % * NFY: Normal Force in Y  [normalized against vectorial load]
    NFY = lat_test.data(:,15);
    %% 
    % * P: Pressure [kPa]
    P = lat_test.data(:,8);
    %% 
    % * RE: Effective Radius [cm]
    RE = lat_test.data(:,7); 
    %% 
    % * RL: Loaded Radius [cm]
    RL = 0.01*lat_test.data(:,6);
    %% 
    % * RST: Road Surface Temperature [°C]
    RST = lat_test.data(:,16);
    %% 
    % * SA: Slip Angle [deg]
    SA = lat_test.data(:,4);
    %% 
    % * SR: Slip Ratio [none]
    SR = lat_test.data(:,21);
    %% 
    % * TSTC: Tire Surface Temperature Center [°C]
    TSTC = lat_test.data(:,18);
    %% 
    % * TSTI: Tire Surface Temperature Inner [°C]
    TSTI = lat_test.data(:,17);
    %% 
    % * TSTO: Tire Surface Temperature Outer [°C]
    TSTO = lat_test.data(:,19);
    %% 
    % * V: Road Velocity [m/s]
    V = 3.6*lat_test.data(:,2);

elseif ext=='.mat'   
    load(fileid);
end

