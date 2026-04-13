classdef PID < handle & matlab.mixin.Copyable
    %PID Combined-form PID controller with second-order filter
    %
    %   obj = PID() creates a PID controller object with default parameters.
    %
    %   obj = PID(Name,Value,...) creates a PID object setting properties
    %   using name-value pairs. Example:
    %       obj = PID("kp",1,"ki",0.2,"Dt",0.01);
    %
    %   Methods:
    %     init(obj,r0,y0,u0,uff0)          Initialize internal states
    %     resetIntegratorLikeStates(obj)    Soft reset integrator-like states
    %     u = control(obj,r,y,uff,uman,utrack,mode)
    %                                       Compute control action
    %     xf2 = filter(obj,y)               Second-order signal filter
    %
    %   Properties (selection):
    %     kp, ki, kd        PID gains
    %     b, c, u0          setpoint weights and bias
    %     Dt, Tf, Tt        sampling and filter/anti-windup times
    %     minlim, maxlim    saturation limits
    %     dumin,  dumanx    rate limitation limits
    %
    %  Internal states:
    %      xu, xus, xr, xdr, xy, xdy, xuff, xf1, xf2              
    %
    %   Example:
    %       pid = PID("kp",2,"ki",0.5,"Dt",0.01);
    %       pid.init(0,0,0,0);
    %       yf =  pid.filter(y);
    %       u  =  pid.control(r,yf,0,0,0,"AUTO");
    %
    %   See also: matlab.mixin.Copyable, help, doc

    % (properties and implementation unchanged except doc comments added)
    properties
        % ===== Controller parameters =====
        kp (1,1) double = 0
        ki (1,1) double = 0
        kd (1,1) double = 0

        b  (1,1) double = 1   % Setpoint weight (P term)
        c  (1,1) double = 1   % Setpoint weight (D term)
        u0 (1,1) double = 0   % Bias/offset (position form without I)

        % ===== Sampling and filter parameters =====
        Dt (1,1) double = 0.01
        Tf (1,1) double = 0.0   % Second-order filter time constant
        Tt (1,1) double = 0.1   % Anti-windup tracking time

        % ===== Saturation and rate limits =====
        minlim (1,1) double = -inf
        maxlim (1,1) double = +inf
        dumin  (1,1) double = -inf   % Minimum du/dt
        dumax  (1,1) double = +inf   % Maximum du/dt
    end

    properties (Access = private)
        % ===== Control internal states =====
        xu   (1,1) double = 0   % Nominal control signal
        xus  (1,1) double = 0   % Last saturated and rate-limited output

        xr   (1,1) double = 0   % Previous reference
        xdr  (1,1) double = 0   % Previous reference derivative
        xy   (1,1) double = 0   % Previous output
        xdy  (1,1) double = 0   % Previous output derivative
        xuff (1,1) double = 0   % Previous feedforward signal

        % ===== Filter states =====
        xf1  (1,1) double = 0
        xf2  (1,1) double = 0

        % ===== Initialization flag =====
        isInitialized (1,1) logical = false
    end

    methods (Access = protected)
        function cp = copyElement(obj)
            % copyElement  Create a copy of the PID object
            %
            %   cp = copyElement(obj) is called by copy(obj) (from
            %   matlab.mixin.Copyable). This default implementation
            %   performs a shallow copy. Override to deep-copy handle
            %   properties if necessary.
            cp = copyElement@matlab.mixin.Copyable(obj); % default shallow copy
        end
    end    

    methods
        function obj = PID(varargin)
            %PID Constructor with optional name-value pairs
            %
            %   obj = PID() creates with defaults.
            %   obj = PID(Name,Value,...) sets properties by name.
            %
            %   Example:
            %     pid = PID("kp",1,"ki",0.2,"Dt",0.01);
            if mod(nargin,2) ~= 0
                error("PID:Constructor","Use name-value pairs.");
            end

            for k = 1:2:nargin
                name = varargin{k};
                val  = varargin{k+1};

                if isprop(obj,name)
                    obj.(name) = val;
                else
                    error("PID:Constructor","Unknown property: %s",name);
                end
            end
        end

        function init(obj,r0,y0,u0,uff0)
            %INIT Initialize internal controller and filter states
            %
            %   init(obj,r0,y0,u0,uff0) sets internal memory to the
            %   provided values. Any arguments can be omitted (defaults
            %   used). Call this at startup or to reset the controller.
            if nargin < 2, r0 = 0; end
            if nargin < 3, y0 = 0; end
            if nargin < 4, u0 = obj.u0; end
            if nargin < 5, uff0 = 0; end

            obj.xr   = r0;
            obj.xy   = y0;

            obj.xdr  = 0;
            obj.xdy  = 0;

            obj.xu   = u0;
            obj.xus  = u0;

            obj.xuff = uff0;

            obj.xf1  = y0;
            obj.xf2  = y0;

            obj.isInitialized = true;
        end

        function resetIntegratorLikeStates(obj)
            %RESETINTEGRATORLIKESTATES Soft reset of integrator-like terms
            %
            %   Leaves controller output memory to avoid bumps but clears
            %   derivative/incremental memory used for integration.
            obj.xdr  = 0;
            obj.xdy  = 0;
            obj.xuff = 0;
        end

        function u = control(obj,r,y,uff,uman,utrack,mode)
            %CONTROL Compute control action (combined-form PID)
            %
            %   u = control(obj,r,y,uff,uman,utrack,mode)
            %
            %   Inputs:
            %     r      reference
            %     y      measured output
            %     uff    feedforward (optional)
            %     uman   manual output (used if mode == "MAN")
            %     utrack tracking output (used if mode == "TRACK")
            %     mode   "TRACK", "MAN", or other (automatic)
            %
            %   Output:
            %     u      control output after saturation and rate limits
            %
            %   Notes:
            %     - If ki == 0 the controller uses position form (no I).
            %     - Anti-windup via back-calculation uses Tt.
            if ~obj.isInitialized
                obj.init(r,y,obj.u0,uff);
            end

            % ----- Rate limits relative to previous saturated output -----
            umin = max(obj.minlim, obj.xus + obj.Dt*obj.dumin);
            umax = min(obj.maxlim, obj.xus + obj.Dt*obj.dumax);

            % ----- Derivative approximations -----
            dr = (r - obj.xr)/obj.Dt;
            dy = (y - obj.xy)/obj.Dt;

            % ----- Control signal update -----
            if exist('mode','var') && mode == "TRACK"
                obj.xu = utrack;
            end

            if exist('mode','var') && mode == "MAN"
                % Manual mode
                u = uman;

            elseif obj.ki == 0.0
                % Position form without integrator
                u = obj.u0 ...
                    + obj.kp*(r - y) ...
                    + obj.kd*(obj.c*dr - dy) ...
                    + uff;

            else
                % Signal increments
                Dr  = r  - obj.xr;
                Dy  = y  - obj.xy;
                Ddr = dr - obj.xdr;
                Ddy = dy - obj.xdy;

                % Increment contributions
                Dup  = obj.kp*(obj.b*Dr - Dy);
                Dui  = obj.ki*obj.Dt*(r - y);                
                Dud  = obj.kd*(obj.c*Ddr - Ddy);
                Duff = uff - obj.xuff;

                % Nominal control signal
                u = obj.xu + Dup + Dui + Dud + Duff;

                % Anti-windup (back-calculation)
                us = max(u,umin);
                us = min(us,umax);
                u  = u - obj.Dt/obj.Tt*(u - us);
            end

            % Store nominal signal before saturation
            obj.xu = u;

            % ----- Apply saturation and rate limits -----
            u = max(u,umin);
            u = min(u,umax);
            obj.xus = u;

            % ----- Update states -----
            obj.xr   = r;
            obj.xdr  = dr;
            obj.xy   = y;
            obj.xdy  = dy;
            obj.xuff = uff;
        end

        function xf2 = filter(obj,y)
            %FILTER Second-order signal filter
            %
            %   xf2 = filter(obj,y) applies a 2nd-order smoothing filter
            %   to input y and returns filtered output xf2. Uses Tf and Dt.
            if ~obj.isInitialized
                obj.init(0,y,obj.u0,0);
            end

            a = obj.Dt/(obj.Tf + 0.5*obj.Dt);

            obj.xf1 = obj.xf1 + a*(y - obj.xf1);
            obj.xf2 = obj.xf2 + a*(obj.xf1 - obj.xf2);

            xf2 = obj.xf2;
        end
    end
end
