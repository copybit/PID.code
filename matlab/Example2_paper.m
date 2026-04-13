close all; clear all; clc;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%% Example 2: P, PD control and rate limitation %%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% This example shows how the proposed code can be used to implement P- or 
% PD-controllers, that is, controllers without integral action. 
% The rate limitation function is also illustrated. 
% We use the same process and controller as presented in Example 1.
% The simulation runs without rate limits by default. To evaluate the rate 
% limitation function, the code lines before the simulation loop must be
% uncommented.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Load process and simulation data
exampleID=2; % ID Example

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
mode="AUTO";          % Controller mode

% State initialization (r0,y0,u0,uff0)
pid1.init(r,r,u0,0);  

% Umcomment the next two lines to evaluate rate limitation 
 %pid1.dumax=0.1/Dt; 
 %pid1.dumin=-pid1.dumax;

% Simulation Loop
for k=Lp+2:1:Tsim/Dt    

    % Simulation of process dynamics
    yc(k)=ap*yc(k-1)+bp*uc(k-Lp-1);        
       
    % Controller type swithing: PI, P, and PI
    if k*Dt>0.333*Tsim+L && k*Dt<=0.675*Tsim+L
        pid1.ki=0;pid1.u0=2;
    else 
        pid1.ki=pid1.kp/Ti;    
    end
        
    % Set-point changes
    if k*Dt<2-L
        r=1;
    elseif k*Dt<=0.498*Tsim+L
        r=3;
    else
        r=1;
    end    
    
    % Controller         
    u  = pid1.control(r,yc(k),uff,uman,utrack,mode);  
               
    uc(k)=u;           
    rc(k)=r;
end

% Plotting graphical results
show_graphical_results;


