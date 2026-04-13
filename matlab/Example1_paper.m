close all; clear all; clc;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%% Example 1: MAN/AUTO %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% This code shows a basic example to show how to switch between manual and
% auto modes. The system starts in manual mode at the beginning of the 
% simulation and then switches to automatic mode in the middle of the 
% simulation time.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Load process and simulation data
exampleID=1; % ID Example

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
uff=0;               % Feedforward signal
uman=0;              % Manual control signal
utrack=0;            % Tracking control signal
mode="MAN";          % Controller mode

% State initialization (r0,y0,u0,uff0)
pid1.init(0,0,0,0);                  

%%%%%% Simulation Loop
for k=Lp+2:1:Tsim/Dt

    % Simulation of process dynamics
    yc(k)=ap*yc(k-1)+bp*uc(k-Lp-1);   
        
    % Control signal switch selection
    if k*Dt<=Tsim/2+L        
        if k*Dt<=2-L
            uman=0;
        else
            uman=1;
        end     
        mode="MAN";
    else
        mode="AUTO";
    end

    % Controller 
    u  = pid1.control(r,yc(k),uff,uman,utrack,mode);    
    uc(k)=u; 

    rc(k)=r;     
end

% Plotting graphical results
show_graphical_results;

