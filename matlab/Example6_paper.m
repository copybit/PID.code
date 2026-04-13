close all; clear all; clc;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%% Example 6: Gain scheduling %%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% This example shows how to use the proposed code for the implementation of 
% gain-scheduling approaches. We consider the classical tank level control 
% problem, which process dynamics is given by the following differential equation:
%
% dy(t)/dt=-a/A\sqrt(2gy(t))+u(t)/A
%
% The gain-scheduling control scheme can be implemented in two different ways: 
% by running all the controllers in parallel and switching among them based 
% on the current operating point; or by using a single controller and updating 
% its parameters based on the current operating point.Both solutions have 
% been used in this example and the code is presented below
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% Load process and simulation data
exampleID=6; % ID Example

process_and_simulation_data;

% Instantiate PID controllers and initialization
pid1 = PID();

pid1.Dt=Dt;          % Sampling time
Ti1=T1;              % Integral time
kp1=T1/(K1*(Tc+L));  % Proportional gain     
ki1=kp1/Ti1;         % Integral gain
kd1=0;               % Derivative gain   
pid1.kp=kp1;
pid1.ki=ki1; 
pid1.kd=kd1;         
pid1.b=1;            % b weigth for set-point weighting
pid1.c=0;            % c weigth for set-point weighting
pid1.Tt=Ti1;         % Tracking constant for back-calculation scheme
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
Ti2=T2;              % Integral time
kp2=T2/(K2*(Tc+L));  % Proportional gain     
ki2=kp2/Ti2;         % Integral gain
kd2=0;               % Derivative gain 
pid2.kp=kp2;             
pid2.ki=ki2; 
pid2.kd=kd2;             
pid2.Tt=Ti2;            % Tracking constant for back-calculation scheme

% Define the third controller
pid3=pid1.copy();
Ti3=T3;              % Integral time
kp3=T3/(K3*(Tc+L));  % Proportional gain     
ki3=kp3/Ti3;         % Integral gain
kd3=0;               % Derivative gain 
pid3.kp=kp3;             
pid3.ki=ki3;   
pid3.kd=kd3;             
pid3.Tt=Ti3;            % Tracking constant for back-calculation scheme

u1=q10;u2=q10;u3=q10;   % Initial value control signals

% State initialization (r0,y0,u0,uff0)
pid1.init(r,h10,u1,0);  
pid2.init(r,h10,u2,0);
pid3.init(r,h10,u3,0); 

% Controllers mode
mode1="AUTO";mode2="TRACK";mode3="TRACK";

% Simulation Loop
for k=Lp+2:1:Tsim/Dt

    % Simulation of process dynamics
    yc(k)=yc(k-1)+Dt*((-a/A)*sqrt(2*g*yc(k-1))+uc(k-1)/A);
    
    % Controller 1        
    uc1(k)  = pid1.control(r,yc(k),uff,uman,utrack,mode1);    
        
    % Controller 2        
    uc2(k)  = pid2.control(r,yc(k),uff,uman,utrack,mode2);    
    
    % Controller 3        
    uc3(k)  = pid3.control(r,yc(k),uff,uman,utrack,mode3);    

    % Simulation scenarios      
    
    if yc(k) < 12
        uc(k)=uc1(k);
        mode1="AUTO";mode2="TRACK";mode3="TRACK";
        
        modes(k)=1;
       
    elseif yc(k) >=12 && yc(k) < 20
        uc(k)=uc2(k);
        mode1="TRACK";mode2="AUTO";mode3="TRACK";
        
        modes(k)=2;
    
     elseif yc(k) >=20 
        uc(k)=uc3(k);

        mode1="TRACK";mode2="TRACK";mode3="AUTO";                      
        
        modes(k)=3;
    end        

    utrack=uc(k);
    
    if k*Dt >= 10 && k*Dt< 200
        r=22;   
    elseif k*Dt>=200 && k*Dt<400
        r=3;
    elseif k*Dt>=400 && k*Dt<600
        r=15;
    end           
          
    rc(k)=r;     
end

%% Second simulation for a single controller updating the parameters based on the operaiting poing
% pid1 is the controller
pid1.kp=kp1;
pid1.ki=ki1;
pid1.kd=kd1;
r=h10;
pid1.init(r,h10,u1,0); 

uff=0;
uman=0;              % Manual control signal
utrack=0;            % Tracking control signal
mode="AUTO";

% Simulation Loop
for k=Lp+2:1:Tsim/Dt

    % Simulation of process dynamics
    ycs(k)=ycs(k-1)+Dt*((-a/A)*sqrt(2*g*ycs(k-1))+ucs(k-1)/A);
   
   % Simulation scenarios
    
    if ycs(k) < 12     
        pid1.kp=kp1;
        pid1.ki=ki1;
        pid1.kd=kd1;               
        
        modess(k)=1;
       
    elseif ycs(k) >=12 && ycs(k) < 20
        pid1.kp=kp2;
        pid1.ki=ki2;
        pid1.kd=kd2; 
        
        modess(k)=2;
    
     elseif ycs(k) >=20  
        pid1.kp=kp3;
        pid1.ki=ki3;
        pid1.kd=kd3;                
        
        modess(k)=3;
    end        
    
    if k*Dt >= 10 && k*Dt< 200
        r=22;
    elseif k*Dt>=200 && k*Dt<400
        r=3;
    elseif k*Dt>=400 && k*Dt<600
        r=15;
    end           
    
   % Controller      
    ucs(k)  = pid1.control(r,ycs(k),uff,uman,utrack,mode);    
               
    rcs(k)=r;     
end

% Plotting graphical results
show_graphical_results; 

