close all; clear all; clc;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%% Example 5: Noise filtering %%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% This example shows how the control signal can be properly filtered by 
% tuning the T_f parameter as input to the filter function. The same process 
% transfer function and PI-controller as used in Example 1 have been used 
% for this simulation, where a white noise signal was added to the process output. 
% Three cases are simulated: without filter, with filter for T_f=0.01T_i, 
% and $T_f=0.1T_i$, respectively. 
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% Load process and simulation data
exampleID=5; % ID Example

process_and_simulation_data;

% Instantiate PID controllers and initialization
pid1 = PID();

pid1.Dt=Dt;          % Sampling time
Ti=T;                % Integral time
pid1.kp=T/(K*(T+L)); % Proportional gain
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
uff=0;
uman=0;              % Manual control signal
utrack=0;            % Tracking control signal
mode="MAN";          % Controller mode

% State initialization (r0,y0,u0,uff0)
pid1.init(0,0,0,0);  

% Define two extra controllers to evaluate different filtering effects
% - pid1: No filter
% - pid2: ilter with Tf=0.1*Ti
% - pid3: ilter with Tf=0.01*Ti

pid2=pid1.copy(); 
pid2.Tf=0.1*Ti;         % Filter with Tf=0.1*Ti
pid3=pid1.copy();
pid3.Tf=0.01*Ti;        % Filter with Tf=0.01*Ti

% Simulation Loop
for k=Lp+2:1:Tsim/Dt

    % Simulation of process dynamics
    yc1(k)=ap*yc1(k-1)+bp*uc1(k-Lp-1)+noisesignal(k);
    yc2(k)=ap*yc2(k-1)+bp*uc2(k-Lp-1)+noisesignal(k);
    yc3(k)=ap*yc3(k-1)+bp*uc3(k-Lp-1)+noisesignal(k);    
   
    % Controller         
    uc1(k)  = pid1.control(r,yc1(k),uff,uman,utrack,mode);    
    yf2 = pid2.filter(yc2(k));
    uc2(k) = pid2.control(r,yf2,uff,uman,utrack,mode);    
    yf3 = pid3.filter(yc3(k));
    uc3(k) = pid3.control(r,yf3,uff,uman,utrack,mode);    

    % AUTO mode and set-point change
    if k*Dt>Tsim/16+L
        mode="AUTO";
        r=3;
    end
    
    rc(k)=r;     
end

% Plotting graphical results
show_graphical_results;







