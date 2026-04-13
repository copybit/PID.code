close all; clear all; clc;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%% Example 7: Selector-override control %%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% This is an example that shows how to handle the tracking state when using
% a selector control approach. The important thing here is to keep the P-part 
% of the controllers as we need their current P-contribution to properly select 
% (for instance, with MIN or MAX selectors) the current control signal.  
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% Load process and simulation data
exampleID=7; % ID Example

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
mode1="AUTO";
mode2="TRACK";

% Define the second controller
pid2=pid1.copy(); 

u1=0;u2=0;    % Initial value control signals

% State initialization (r0,y0,u0,uff0)
pid1.init(r1,r1,u1,0);  
pid2.init(r2,r2,u2,0); 

% Simulation Loop
for k=Lp+2:1:Tsim/Dt
        
    % Simulation of process dynamics
    yc1(k)=ap*yc1(k-1)+bp*(uc(k-Lp-1)-v(k));    
    yc2(k)=ap*yc2(k-1)+bp*uc(k-Lp-1);   
    
    % Controller 1             
    uc1(k)  = pid1.control(r1,yc1(k),uff,uman,utrack,mode1);    
        
    % Controller 2        
    uc2(k)  = pid2.control(r2,yc2(k),uff,uman,utrack,mode2);    
   
    % Selector
    if uc1(k) < uc2(k)
        uc(k)=uc1(k);
        mode1="AUTO";mode2="TRACK";
                
    elseif uc2(k) <= uc1(k)              
        uc(k)=uc2(k);  
        mode1="TRACK";mode2="AUTO";
    end

    utrack=uc(k);
end

% Plotting graphical results
show_graphical_results;      