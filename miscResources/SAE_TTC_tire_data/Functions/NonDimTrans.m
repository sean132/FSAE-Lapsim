% Copyright 2017 The MathWorks, Inc.

function [CS,mu,S_H,S_V,Slip_bar,F_bar] = NonDimTrans(F,NF,Slip,ET,FZ)
%[CS,mu,S_H,S_V,Slip_bar,F_bar] = NonDimTrans(F,NF,Slip,ET,FZ)
%==================
% [CS,mu,S_H,S_V,Slip_bar,F_bar] = NonDimTrans(F,NF,Slip,ET,FZ)
%==================
%This function takes in given lateral/longitudinal force data, slip
%angle/ratio and time data and averages the sweeps to output a slip
%stiffness and force coefficient to be used in a non-dimensional model. The
%parameters are then shifted and transformed to non-dimensional space (bar
%quantities)
%==================
% INPUT PARAMETERS
%==================
%F = Force vector (N)
%NF = non-dimensional force coefficient
%Slip = Slip quantity vector (deg or -)
%ET = time vector (s)
%FZ = Vertical load (N)
%==================
% OUTPUT PARAMETERS
%==================
%CS = Slip Stiffness parameter
%mu = force coefficient
%S_H = horizontal shift according to polynomial fit
%S_V = vertical shift according to polynomial fit
%Slip_bar = Non-dimensional slip angle/ratio
%F_bar = Non-dimensional Force
%------------------

% set defaults outputs for failure modes
CS=0;
mu=0;
S_H=0;
S_V=0;
Slip_bar=0;
F_bar=0;

if(numel(Slip) > 1)
    %% Seperate Data in positive and negative sweeps:
    dSlip_dt = diff(Slip)./diff(ET);
    dSlip_dt(end+1) = dSlip_dt(end); %assume end difference value is the same as previous (forward differencing)
    
    Pos_sweep = dSlip_dt>0;
    Neg_sweep = dSlip_dt<0;
    
    F_plus = F(Pos_sweep);NF_plus = NF(Pos_sweep);
    Slip_plus = Slip(Pos_sweep);
    Time_plus = ET(Pos_sweep);
    
    F_minus = F(Neg_sweep);
    NF_minus = NF(Neg_sweep);
    Slip_minus = -Slip(Neg_sweep);
    Time_minus = ET(Neg_sweep); %reflect across y axis the negative sweep (positive SA gives positive force)
    
    %Average data across positive and negative sweeps to have equally weighted
    %data on final average.
    F_pos = AverageSame(F_plus,Slip_plus);
    NF_pos = AverageSame(NF_plus,Slip_plus);
    F_neg = AverageSame(F_minus,Slip_minus);
    NF_neg = AverageSame(NF_minus,Slip_minus);
    
    %Force Coefficient
    mu_pos = max(abs(NF_pos));
    mu_neg = max(abs(NF_neg));
    mu = mean([mu_pos;mu_neg]);
    
    %Cornering Stiffness
    %define range based on max force coefficient
    [mu_max,Ind_max] = max(abs(NF));
    
    %This function can also be used for lateral Tire Data, differentiate between
    %Slip Angle for lateral Force FY and Slip Ratio for longtitudinal Force FX
    if max(abs(Slip))>2 %FY
        Slip_limit = abs(0.3*Slip(Ind_max));
        Slip_range_pos = (-Slip_limit<Slip_plus)&(Slip_plus<Slip_limit);
        Slip_range_neg = (-Slip_limit<Slip_minus)&(Slip_minus<Slip_limit);
        F_fit_pos = F_plus(Slip_range_pos); Slip_fit_pos = Slip_plus(Slip_range_pos);
        F_fit_neg = F_minus(Slip_range_neg); Slip_fit_neg = Slip_minus(Slip_range_neg);
    else %FX
        %Find rough x-intercept
        Slip_minus = -Slip_minus;
        Slip_int = mean(Slip_plus((F_plus<150)&(F_plus>-150)));
        Slip_limit = abs(0.3*Slip(Ind_max));
        Slip_range_pos = (-(Slip_limit-Slip_int)<Slip_plus)&(Slip_plus<(Slip_limit+Slip_int));
        Slip_range_neg = (-(Slip_limit-Slip_int)<Slip_minus)&(Slip_minus<(Slip_limit+Slip_int));
        F_fit_pos = F_plus(Slip_range_pos); Slip_fit_pos = Slip_plus(Slip_range_pos);
        F_fit_neg = F_minus(Slip_range_neg); Slip_fit_neg = Slip_minus(Slip_range_neg);
    end
    
    if max(abs(Slip))>2 %must be slip angle if true - FY
        %Check that positive slip angle results in positive Force(only FY)
        Check_pos = Slip_fit_pos>0.2; %positive slip range
        Check_neg = Slip_fit_neg>0.2; %positive slip range
        
        if mean(F_fit_pos(Check_pos))<0
            Slip_fit_pos = -Slip_fit_pos;
        elseif mean(F_fit_neg(Check_neg))<0
            Slip_fit_neg = -Slip_fit_neg;
        end
    end
    
    %Fit a 3rd order polynomial to both sets of data
    [Pos,~,mupos] = polyfit(Slip_fit_pos,F_fit_pos,3);
    [Neg,~,muneg] = polyfit(Slip_fit_neg,F_fit_neg,3);
    
    if numel(Slip_fit_pos) < 4 || numel(F_fit_pos) < 4 || numel(F_fit_neg) < 4
        return
    end
    
    Fit2=fit(Slip_fit_pos,F_fit_pos,'poly3','Normalize','off');
    Fit3=fit(Slip_fit_neg,F_fit_neg,'poly3','Normalize','off');
        
    
    %Average both
    a = mean([Fit2.p1 Fit3.p1]);
    b= mean([Fit2.p2 Fit3.p2]);
    c= mean([Fit2.p3 Fit3.p3]);
    d= mean([Fit2.p4 Fit3.p4]);
    
    %Find vertical and horizontal shift
    Poly = @(x) a*x.^3+b*x.^2+c*x+d;
    if max(abs(Slip))>2
        S_V = d;
        Zero_poly = @(x)a*x.^3+b*x.^2+c*x+d-S_V;
        S_H = fzero(Zero_poly,0); %horizontal shift corresponds to vertically shifted function x intercept .
        CS = (3*a*(S_H).^2+2*b*(S_H)+c)*180/pi; %Slip stiffness used is N/rad
        
    else
        S_V = (abs(max(F))-abs(min(F)))/2; %assume only horizontal shift for Fx
        Zero_poly = @(x)a*x.^3+b*x.^2+c*x+d-S_V; %shift function vertically then find horizontal shift.
        S_H = fzero(Zero_poly,Slip_int); %horizontal shift corresponds to vertically shifted function x intercept .
        CS = (3*a*(S_H).^2+2*b*(S_H)+c); %Slip stiffness used is N/unit slip
    end
    
    %% use me for debugging...
    %Shifted Transforms and Non-dimensional quantities
    % figure
    % x = linspace(min(Slip_fit_neg),max(Slip_fit_neg),200);
    % plot(Slip_fit_pos,F_fit_pos,'.',Slip_fit_neg,F_fit_neg,'.',x,Poly(x),S_H,S_V,'ok','LineWidth',2,'MarkerSize',20);
    % title('F_x vs. \kappa','FontSize',20);
    % ylabel('F_x (N)','FontSize',18);
    % xlabel('\kappa','FontSize',18);
    % hleg = legend('Positive Sweep','Negative Sweep','Polynomial Fit','Shift Point');
    % set(hleg,'FontSize',18,'Location','NorthWest');
    % set(gca,'FontSize',14)
    % grid on
    
    %% NonDimensionalize the values
    FZ0 = 50*4.448*round(mean(FZ/4.448)/50); %units N
    S_V_bar = S_V/FZ0;
    CS_bar = CS./FZ;
    S_H_bar = S_H*max(CS_bar);
    F_shift = F-FZ0*S_V_bar;
    Slip_shift = Slip-S_H_bar./CS_bar;
    
    %find Force where shifted mu is maximum
    [F_shift_max,Ind_shift_max] = max(abs(F_shift)); %%
    FZ_max = FZ(Ind_shift_max);
    mu_shift = abs(F_shift_max/FZ_max);
    
    %F_bar = 1/mu_shift.*(F./FZ-S_V_bar);
    F_bar = 1/mu_shift.*(F_shift./FZ);
    
    if max(Slip)>2 %check what the slip quantity is, slip angle (degrees) or slip ratio (-)
        Slip_bar = (CS_bar)./mu_shift.*(tan((Slip-S_H_bar./(CS_bar))*pi/180));
    else
        Slip_bar = 1/mu_shift*(CS_bar.*Slip-S_H_bar);
        
    end
    
    
else
    %This can also be intended.
    warning('There might be no corresponding slip value');
end

end



