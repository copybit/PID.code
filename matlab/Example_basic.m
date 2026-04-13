close all; clear all; clc;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%% Example: basic control loop %%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% To illustrate how the code can be used for simulation purposes, this 
% example shows the code for a basic control loop using the filtering 
% capabilities of the proposed PID control code.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Load process and simulation data
exampleID=0; % ID Example

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

% Control Loop Simulation
for k=Lp+2:1:Tsim/Dt

    % Simulation of process dynamics
    yc(k)=ap*yc(k-1)+bp*uc(k-Lp-1)+noisesignal(k);
   
    % Controller     
    yf = pid1.filter(yc(k));
    u  = pid1.control(r,yf,uff,uman,utrack,mode);    
                   
    uc(k)=u;    

    % AUTO mode
    if k*Dt>Tsim/16+L
        mode="AUTO";
    end    
    
    % Set-point change
    if k*Dt>2-L
        r=3;
    end
    
    rc(k)=r;     
end

% Plotting graphical results
show_graphical_results;