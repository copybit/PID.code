close all; clear all; clc;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%% Example 4: Feedforward and anti-windup %%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% This example focuses on analyzing the anti-windup solutions included in 
% the PID code for the control signal saturation problem, and also the 
% combination of the PID controller with a feedforward compensator to deal 
% with measurable disturbances. The effect of anti-windup clamping and t
% racking solutions implemented will be analyzed and compared. Moreover, 
% the capability of the new PID code that includes the feedforward control 
% signal inside the PID controller will also be explored.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% Load process and simulation data
exampleID=4; % ID Example

process_and_simulation_data;

% Instantiate PID controllers and initialization
pid1 = PID();

pid1.Dt=Dt;          % Sampling time
Ti=T;                % Integral time
pid1.kp=T/(K*(0.2*T+L)); % Proportional gain
pid1.ki=pid1.kp/Ti;  % Integral gain
pid1.kd=0;           % Derivative gain
pid1.b=1;            % b weigth for set-point weighting
pid1.c=0;            % c weigth for set-point weighting
pid1.Tt=Ti;          % Tracking constant for back-calculation scheme
pid1.maxlim=Inf;     % Maximum control limit signal
pid1.minlim=-Inf;    % Minimum control limit signal
pid1.dumax=Inf;      % Maximum Increment control limit signal
pid1.dumin=-Inf;     % Minimum Increment control limit signal
pid1.Tf=Dt;          % Filter Time constant

% Input signals
uman=0;              % Manual control signal
utrack=0;            % Tracking control signal
mode="AUTO";          % Controller mode

% State initialization (r0,y0,u0,uff0)
pid1.init(0,0,0,0);  

% Define three extra controllers to evaluate different anti-windup
% techqniues:
% - pid1: with no saturation limits
% - pid2: with only external satuartion limits
% - pid3: with satuartion limits using clamping
% - pid4: with satuartion limits using tracking

umax=3.5;umin=-3.5;
pid2=pid1.copy(); 
pid3=pid1.copy();
pid3.maxlim=umax;pid3.minlim=umin;pid3.Tt=Dt;
pid4=pid1.copy();
pid4.maxlim=umax;pid4.minlim=umin;

% Feedforward parameters
Kff = Kd/K;
Tz = T;
Tp = Td;
Lff = Ld-L;

% Discrete feedforward parameters
aff=exp(-Dt/Tp); bff=exp(-Dt/Tz); affd=Kff*(1-aff)/(1-bff);
bffd=affd*bff; Lffd=Lff/Dt;

% Simulation Loop
for k=Lp+2:1:Tsim/Dt
    
    % Simulation of process dynamics
    yc1(k)=ap*yc1(k-1)+bp*uc1(k-Lp-1);
    yc2(k)=ap*yc2(k-1)+bp*uc2(k-Lp-1);
    yc3(k)=ap*yc3(k-1)+bp*uc3(k-Lp-1);
    yc4(k)=ap*yc4(k-1)+bp*uc4(k-Lp-1);    

    % Disturbance dynamics
    yd(k)=apd*yd(k-1)+bpd*ud(k-Lpd-1);    
    
    % Total process output
    y1(k)=yc1(k)+yd(k);
    y2(k)=yc2(k)+yd(k);
    y3(k)=yc3(k)+yd(k);
    y4(k)=yc4(k)+yd(k);
  
    % Disturbance signal change   
    if k*Dt> 45+L %1+L
        ud(k)=1.5;
    end
    
    % Feedforward compensator   
    uff(k)=(aff*uff(k-1)+affd*ud(k-Lffd)-bffd*ud(k-Lffd-1));
    
    % Controller         
    uc1(k) = pid1.control(r,y1(k),-uff(k),uman,utrack,mode);
    uc2(k) = pid2.control(r,y2(k),-uff(k),uman,utrack,mode);
    uc3(k) = pid3.control(r,y3(k),-uff(k),uman,utrack,mode);
    uc4(k) = pid4.control(r,y4(k),-uff(k),uman,utrack,mode);

    % External saturation for case 2
    uc2(k)=max(min(uc2(k),umax),umin);

    if k*Dt>2-L && k*Dt<20+L   
        r=4.5;
    elseif k*Dt>=20+L      
        r=0;
    end     
    
    rc(k)=r;     
end

% Plotting graphical results
show_graphical_results;