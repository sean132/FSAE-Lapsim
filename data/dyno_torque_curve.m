%% List of Torque Curves
%different arrays are for different engine tunes
%recorded as [rpm, torque(ft-lbs)]

close
spark_tune = [  %preliminary spark tune jake made B19 winterbreak
6000	22.5535
6500	24.2606
7000	25.6219
7500	26.7192
8000	27.1255
8500	26.9020
9000	25.8048
9500	25.6219
10000	24.7076
10500	22.249
11000	20.8674
11500	20.461
12000	19.5263
12500	18.0837];

Supposed_18Comp_Curve = [ %torque curve given to jake before B18 comp. bad load cell calibration? or bluntly false data?
6000	24.03454457
6500	25.25664005
7000	25.05295747
7500	25.25664005
8000	25.66400521
8500	25.05295747
9000	23.6271794
9500	22.20140134
10000	21.59035359
10500	20.57194069
11000	19.75721036
11500	18.53511488
12000	15.68355874
12500	16.29460649];

Supposed_17Comp_Curve = [ %supposed B17 torque curve. basically after seeing the supposed B18 vs actual B18 I lost all faith in anything engine had done previously.
6000	15.43116946
6500	15.80753944
7000	18.06575936
7500	17.31301939
9000	23.14675419
9500	22.58219921
10000	22.01764422
10500	21.26490425
11000	20.32397928
11500	19.38305432
12000	17.87757437
12500	15.43116946
13000	14.1138745];

comp18_actual = [ %jake recorded this torque curve on the dyno with the comp map
6000	19.8921
6500	22.0458
7000	22.1678
7500	23.8339
8000	23.8542
8500	23.5494
9000	22.249
9500	21.7411
10000	20.7454
10500	19.6076
11000	17.9821
11500	17.1897
12000	16.6614
12500	15.2391];

Ricardo_E85 = [ %davis ricardo simulation for E85 
6000	24.2
6500	26.5
7000	28.3
7500	28.7
8000	28.8
8500	28.3
9000	27.3
9500	25.9
10000	24.4
10500	23.1
11000	21.9
11500	20.8
12000	19.6
12500	18.6
13000	17.6];

Ricardo_100 = [ %davis ricardo simulation for 100 octane
6000	22
6500	23.8
7000	26.1
7500	27
8000	26.9
8500	26.7
9000	26.1
9500	24.9
10000	23.64
10500	22.4
11000	21.2
11500	20.1
12000	19.1
12500	18.2
13000	17.2];

stock_2016 = [ %taken from the internet by an outside vendor that dyno'd a stock 2016 ktm 350 bike
%3889	18.117
%4033	18.194
%4176	18.328
%4320	18.487
%4464	18.615
%4607	18.867
%4751	19.175
%4895	19.493
%5020	19.876
%5182	20.325
%5326	20.848
%5469	21.449
%5613	22.035
%5757	22.635
%5900	23.262
6044	23.826
6188	24.289
6331	24.833
6475	25.403
6619	25.885
6762	26.255
6906	26.522
7050	26.604
7193	26.702
7337	26.825
7481	26.974
7624	27.041
7768	27.056
7912	27.246
8055	27.467
8199	27.498
8343	27.313
8486	27.154
8630	27.154
8774	27.154
8917	26.994
9061	26.799
9205	26.537
9348	26.260
9492	25.957
9636	25.490
9779	25.156
9923	25.120
10067	25.100
10210	24.864
10354	24.448
10498	24.186
10641	24.160
10785	24.052
10929	23.729
11072	23.477
11216	23.272
11360	23.072
11503	22.892
11647	22.738
11791	22.548
11934	22.271
12078	22.014
12222	21.747
12365	21.506
12509	21.341
12653	21.074
12796	20.736
12940	20.453
];


stock450 = [ %taken from  an outside vendor  that dyno'd a stock ktm 450 bike
% 4925, 25.66
% 5024, 25.42
% 5133, 26.00
% 5237, 26.49
% 5341, 27.14
% 5445, 27.65
% 5624, 28.21
% 5728, 28.82
% 5832, 29.50
% 5936, 30.20
6040, 30.83
6144, 31.39
6248, 31.83
6352, 32.16
6456, 32.41
6560, 32.67
6664, 32.99
6768, 33.28
6872, 33.50
6976, 33.59
7080, 33.62
7185, 33.75
7289, 33.92
7393, 34.05
7497, 33.95
7601, 33.78
7705, 33.64
7809, 33.58
7913, 33.51
8017, 33.33
8121, 33.06
8225, 32.81
8329, 32.61
8433, 32.40
8537, 32.07
8641, 31.71
8745, 31.44
8849, 31.16
8953, 30.75
9057, 30.30
9161, 29.99
9265, 29.77
9369, 29.51
9473, 29.30
9577, 29.09
9681, 28.73
9785, 28.30
9889, 27.91
9993, 27.58
1.010e+4, 27.32
1.020e+4, 27.03
1.030e+4, 26.53
1.041e+4, 25.96
1.051e+4, 25.60
1.062e+4, 25.27
1.072e+4, 24.87
1.082e+4, 24.52
1.093e+4, 24.16
1.103e+4, 23.82
1.114e+4, 23.21
1.124e+4, 22.91
1.134e+4, 22.74
1.145e+4, 22.33
1.151e+4, 21.95];

plot(Supposed_17Comp_Curve(:,1),Supposed_17Comp_Curve(:,2))
hold on
plot(Supposed_18Comp_Curve(:,1),Supposed_18Comp_Curve(:,2))
hold on
plot(comp18_actual(:,1),comp18_actual(:,2))
hold on
plot(spark_tune(:,1),spark_tune(:,2))
hold on
plot(Ricardo_E85(:,1),Ricardo_E85(:,2))
hold on
plot(Ricardo_100(:,1),Ricardo_100(:,2))
hold on
plot(stock_2016(:,1),stock_2016(:,2))
hold on
plot(stock450(:,1),stock450(:,2))

title('Torque Curve Comparison')
ylabel('Torque (ft-lbs)')
xlabel('RPM')
legend('Supposed B17 Comp','Supposed B18 Comp', 'Actual B18 Comp', 'Preliminary Spark Tune','Ricardo E85','Ricardo 100 Octane','Stock 350','Stock 450') 
%%
close
plot(Supposed_17Comp_Curve(:,1),Supposed_17Comp_Curve(:,2).*Supposed_17Comp_Curve(:,1)/5252)
hold on
plot(Supposed_18Comp_Curve(:,1),Supposed_18Comp_Curve(:,2).*Supposed_18Comp_Curve(:,1)/5252)
hold on
plot(comp18_actual(:,1),comp18_actual(:,2).*comp18_actual(:,1)/5252)
hold on
plot(spark_tune(:,1),spark_tune(:,2).*spark_tune(:,1)/5252)
hold on
plot(Ricardo_E85(:,1),Ricardo_E85(:,2).*Ricardo_E85(:,1)/5252)
hold on
plot(Ricardo_100(:,1),Ricardo_100(:,2).*Ricardo_100(:,1)/5252)
hold on
plot(stock_2016(:,1),stock_2016(:,2).*stock_2016(:,1)/5252)
hold on
plot(stock450(:,1),stock450(:,2).*stock450(:,1)/5252)

title('HorsePower Curve Comparison')
ylabel('Power (HP)')
xlabel('RPM')
legend('Supposed B17 Comp','Supposed B18 Comp', 'Actual B18 Comp', 'Preliminary Spark Tune','Ricardo E85','Ricardo 100 Octane','Stock 350','Stock 450') 
%% This code is used for formating data from the ecu
% takes in LoadCell and EngineRPM and creates plot for comparison purposes

% close
% primary = 3.04;
% fifth = .95;
% gear = 12/13;
% total_reduction = primary*fifth*gear;
% load_2_ft_lb = 6.5/12;
% Torque = (LoadCell*load_2_ft_lb/total_reduction);
% Load2Torque = @(lb) lb*load_2_ft_lb/total_reduction;
% HorsePower = Torque.*EngineRPM/5252;
% plot(EngineRPM,Torque,'x')
% %hold on 
% %plot(EngineRPM,HorsePower,'x')
% Tmax = max(Torque)
% Hmax = max(HorsePower)