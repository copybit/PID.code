switch exampleID
    case 0 % Deafult example - Simple code with noise and filtering

        %%%%%% Process parameters        
        K=1;            % Static gain
        T=1;            % Time constant
        L=0.5;          % Time delay
        
        y0=0;           % Initial value of the process ouput
        u0=0;           % Initial value of the controller ouput
        
        %%%%%% Simulation paramters
        Tsim=20+L;      % Simulation time  
        
        Dt=0.01;        % Sampling time
        
        % Discrete dynamics for simulation of the process model
        ap=exp(-Dt/T); bp=K*(1-ap); Lp=L/Dt;
        
        % Array for simulation
        uc=zeros(Tsim/Dt,1);    % Control signal array for b=1
        yc=zeros(Tsim/Dt,1);    % Process output array for b=1
        rc=zeros(Tsim/Dt,1);     % Reference signal array
        r=0;                     % Initial reference value
        
        % Noise signal
        SNRdb = 5; %Given SNRdb
        variance = 1/(Dt*(10^(SNRdb/10))); 
        load semilla
        rng(s); 
        W = sqrt(variance).*randn(Tsim/Dt); %Gaussian white noise W
        noisesignal=0.01*W;

    case 1 % Example 1 - From man to auto

        %%%%%% Process parameters
        
        K=1;            % Static gain
        T=1;            % Time constant
        L=0.5;          % Time delay
        
        y0=0;           % Initial value of the process ouput
        u0=0;           % Initial value of the controller ouput
        
        %%%%%% Simulation paramters
        Tsim=20+L;      % Simulation time  
        
        Dt=0.01;        % Sampling time
        
        % Discrete dynamics for simulation of the process model
        ap=exp(-Dt/T); bp=K*(1-ap); Lp=L/Dt;
        
        % Array for simulation
        uc=zeros(Tsim/Dt,1);    % Control signal array for b=1
        yc=zeros(Tsim/Dt,1);    % Process output array for b=1
        rc=zeros(Tsim/Dt,1);     % Reference signal array
        r=3;                     % Initial reference value    

    case 2 % Example 2 - P, PD control and rate limitation

        %%%%%% Process parameters
        
        K=1;            % Static gain
        T=1;            % Time constant
        L=0.5;          % Time delay
        
        y0=0;           % Initial value of the process ouput
        u0=1;           % Initial value of the controller ouput
        
        %%%%%% Simulation paramters
        Tsim=30+L;      % Simulation time  
        
        Dt=0.1;        % Sampling time
        
        % Discrete dynamics for simulation of the process model
        ap=exp(-Dt/T); bp=K*(1-ap); Lp=L/Dt;
        
        % Array for simulation
        uc=ones(Tsim/Dt,1);     % Control signal array for b=1
        yc=ones(Tsim/Dt,1);     % Process output array for b=1
        rc=ones(Tsim/Dt,1);     % Reference signal array
        r=1;                    % Initial reference value            

    case 3 % Example 3 - Set-point weighting

        %%%%%% Process parameters
        
        K=1;            % Static gain
        T=1;            % Time constant
        L=0.5;          % Time delay
        
        y0=0;           % Initial value of the process ouput
        u0=0;           % Initial value of the controller ouput
        
        %%%%%% Simulation paramters
        Tsim=20+L;   % Simulation time  

        Dt=0.01;        % Sampling time        
        
        % Discrete dynamics for simulation of the process model
        ap=exp(-Dt/T); bp=K*(1-ap); Lp=L/Dt;
        
        % Array for simulation
        uc1=zeros(Tsim/Dt,1);    % Control signal array for b=1
        uc2=zeros(Tsim/Dt,1);    % Control signal array for b=0.5
        uc3=zeros(Tsim/Dt,1);    % Control signal array for b=0
        yc1=zeros(Tsim/Dt,1);    % Process output array for b=1
        yc2=zeros(Tsim/Dt,1);    % Process output array for b=0.5
        yc3=zeros(Tsim/Dt,1);    % Process output array for b=0
        rc=zeros(Tsim/Dt,1);     % Reference signal array
        r=0;                     % Initial reference value
        dist=0;                  % Disturbance signal

    case 4 % Example 4 - Feedforward and anti-windup

        %%%%%% Process parameters
        
        % Pu dynamics 
        K=1;        % Static gain
        T=3;        % Time constant
        L=0.5;      % Time delay
        
        % Pd dynamics
        Kd=2;       % Static gain
        Td=1;       % Time constant
        Ld=0.5;     % Time delay
        
        y0=0;       % Initial value of the process ouput
        u0=0;       % Initial value of the controller ouput
        
        umax=7;     % Maximum control limit signal
        umin=0;     % Minimum control limit signal
        dumax=Inf;  % Maximum Increment control limit signal 
        dumin=-Inf; % Minimum Increment control limit signal
                
        Dt=0.01;    % Sampling time        
        
        %%%%%% Simulation paramters
        Tsim=70+L;   % Simulation time  
        
        % Discrete dynamics for simulation of the process model
        
        % Pu 
        ap=exp(-Dt/T); bp=K*(1-ap); Lp=L/Dt;
        
        % Pu
        apd=exp(-Dt/Td); bpd=Kd*(1-apd); Lpd=Ld/Dt;
        
        % Array for simulation
        uff=zeros(Tsim/Dt,1);       % Feedforward control signal array
        uc1=zeros(Tsim/Dt,1);        % Control signal array without saturation - case 1
        uc2=zeros(Tsim/Dt,1);        % Control signal array without anti-windup - case 2
        uc3=zeros(Tsim/Dt,1);        % Control signal array with internal feedforward + clamping - case 3
        uc4=zeros(Tsim/Dt,1);        % Control signal array with internal feedforward + tracking - case 4
        utotal1=zeros(Tsim/Dt,1);   % Total control singal  - case 1
        utotal2=zeros(Tsim/Dt,1);   % Total control singal  - case 2
        utotal3=zeros(Tsim/Dt,1);   % Total control singal  - case 3
        utotal4=zeros(Tsim/Dt,1);   % Total control singal  - case 4
        yc1=zeros(Tsim/Dt,1);        % Process output array - case 1
        yc2=zeros(Tsim/Dt,1);        % Process output array - case 2
        yc3=zeros(Tsim/Dt,1);        % Process output array - case 3
        yc4=zeros(Tsim/Dt,1);        % Process output array - case 4
        y1=zeros(Tsim/Dt,1);        % Total process output Pu+Pd - case 1
        y2=zeros(Tsim/Dt,1);        % Total process output Pu+Pd - case 2
        y3=zeros(Tsim/Dt,1);        % Total process output Pu+Pd - case 3
        y4=zeros(Tsim/Dt,1);        % Total process output Pu+Pd - case 4
        yd=zeros(Tsim/Dt,1);        % Disturbance dynamics array 
        ud=zeros(Tsim/Dt,1);        % Disturbance signal array
        rc=ones(Tsim/Dt,1);         % Reference signal array
        r=0;                  

    case 5 % Example 5 - Filtering measurement noise

        %%%%%% Process parameters
        
        K=1;            % Static gain
        T=1;            % Time constant
        L=0.5;          % Time delay
        
        y0=0;           % Initial value of the process ouput
        u0=0;           % Initial value of the controller ouput
                        
        %%%%%% Simulation paramters
        Tsim=20+L;      % Simulation time  

        Dt=0.01;    % Sampling time        
        
        % Discrete dynamics for simulation of the process model
        ap=exp(-Dt/T); bp=K*(1-ap); Lp=L/Dt;
        
        % Array for simulation
        uc1=zeros(Tsim/Dt,1);    % Control signal array for C1
        uc2=zeros(Tsim/Dt,1);    % Control signal array for C2
        uc3=zeros(Tsim/Dt,1);    % Control signal array for C3        
        yc1=zeros(Tsim/Dt,1);    % Process output array for C1
        yc2=zeros(Tsim/Dt,1);    % Process output array for C2
        yc3=zeros(Tsim/Dt,1);    % Process output array for C3        
        rc=zeros(Tsim/Dt,1);     % Reference signal array
        r=0;                     % Initial reference value
        
        % Noise signal
        SNRdb = 5; %Given SNRdb
        variance = 1/(Dt*(10^(SNRdb/10))); 
        load semilla
        rng(s); 
        W = sqrt(variance).*randn(Tsim/Dt); %Gaussian white noise W
        noisesignal=0.01*W;        

    case 6 % Example 6 - Adaptive control of tank level

        %%%% Tank Process parameters
        A=390;a=2.15;g=983;
        
        % Operating point 1
        h10=4;
        T1=(A/a)*sqrt((2*h10/g));   % Time constant
        K1=T1/A;                    % Static gain
        L=0;                        % Time delay
        
        q10=a*sqrt(2*g*h10);        % Starting operating point
        
        % Operating point 2
        h20=12;
        T2=(A/a)*sqrt((2*h20/g));   % Time constant
        K2=T2/A;                    % Static gain
        L=0;                        % Time delay
        
        % Operating point 3
        h30=20;
        T3=(A/a)*sqrt((2*h30/g));   % Time constant
        K3=T3/A;                    % Static gain
        L=0;                        % Time delay       

        %%%%%% Simulation paramters
        Tsim=600;           % Simulation time
        r=h10;              % Initial reference value

        Dt=0.01;    % Sampling time        
        
        % Delay
        Lp=ceil(L/Dt);
        
        % Array for simulation for paralell controllers
        q10=a*sqrt(2*g*h10);
        uc=ones(Tsim/Dt,1)*q10;
        yc=ones(Tsim/Dt,1)*h10;
        rc=ones(Tsim/Dt,1)*h10;
        uc1=ones(Tsim/Dt,1)*q10;
        uc2=ones(Tsim/Dt,1)*q10;
        uc3=ones(Tsim/Dt,1)*q10;
        modes=ones(Tsim/Dt,1);

        % Array for simulation for single controller
        ucs=ones(Tsim/Dt,1)*q10;  
        ycs=ones(Tsim/Dt,1)*h10;
        rcs=ones(Tsim/Dt,1)*h10;       
        modess=ones(Tsim/Dt,1);

        % Global closed-loop time constant
        Tc=(T1+T2+T3)/3;
        
    case 7 % Example 7 - Selectors

        %%%%%% Process parameters
        
        K=1;        % Static gain
        T=1;        % Time constant
        L=0.5;      % Time delay
        
        %%%%%% Simulation paramters
        Tsim=150;   % Simulation time  
        
        Dt=0.01;    % Sampling time
        
        % Discrete dynamics for simulation of the process model
        ap=exp(-Dt/T); bp=K*(1-ap); Lp=L/Dt;
        
        % Array for simulation
        r1=0.3;                  % Initial reference value for r1
        r2=0.5;                  % Initial reference value for r2
        
        uc1=zeros(Tsim/Dt,1);    % Control signal array controller 1
        uc2=zeros(Tsim/Dt,1);    % Control signal array controller 2
        uc=zeros(Tsim/Dt,1);     % Final control signal array 
        yc1=ones(Tsim/Dt,1)*r1;  % Process output array y1
        yc2=ones(Tsim/Dt,1)*r2;  % Process output array y2
        rc1=ones(Tsim/Dt,1)*r1;  % Reference signal for y1
        rc2=ones(Tsim/Dt,1)*r2;  % Reference signal for y2
        
        load v;                  % Disturbance signal profile      

    otherwise % Deafult example 
        error('Wrong Example ID');            
end

