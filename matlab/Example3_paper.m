close all; clear all; clc;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%% Example 3: Set-point weightihg %%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% The PID code used here includes the capabilities for setpoint handling 
% to obtain a smoother control signal when strong changes of setpoints 
% (like step changes) are required. To show this idea, in this example we 
% use the same process and controller as presented in Example 1 to show the 
% effect of the b parameter on the responses of the control system. 
% Three different cases are analyzed for b=0, b=0.5, and b=1. 
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% Load process and simulation data
exampleID=3; % ID Example

process_and_simulation_data;

% Instantiate PID controllers and initialization
pid1 = PID();

% PID with b=1
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
mode="AUTO";          % Controller mode

% State initialization (r0,y0,u0,uff0)
pid1.init(0,0,0,0);  

% Define two extra controllers only changing b parameters
% PID with b=0.5
pid2=pid1.copy(); pid2.b=0.5;
% PID with b=0
pid3=pid1.copy(); pid3.b=0;
    
% Simulation Loop
for k=Lp+2:1:Tsim/Dt

    % Simulation of process dynamics
    yc1(k)=ap*yc1(k-1)+bp*(uc1(k-Lp-1)+dist);
    yc2(k)=ap*yc2(k-1)+bp*(uc2(k-Lp-1)+dist);
    yc3(k)=ap*yc3(k-1)+bp*(uc3(k-Lp-1)+dist);
   
    % Controller     
    u1  = pid1.control(r,yc1(k),uff,uman,utrack,mode);
    u2  = pid2.control(r,yc2(k),uff,uman,utrack,mode);
    u3  = pid3.control(r,yc3(k),uff,uman,utrack,mode);
                   
    uc1(k)=u1;uc2(k)=u2;uc3(k)=u3;         
    
    % Disturbance signal change
    if k*Dt>Tsim/2+L
        dist=1;
    end
    
    % Set-point change
    if k*Dt>2-L
        r=3;
    end
    
    rc(k)=r;     
end

% Plotting graphical results
show_graphical_results;



