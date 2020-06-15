classdef ContinuousLpvDynamics < handle
    %CONTINUOUSLPVDYNAMICS Convenience class for the simulation of LPV
    %systems
    %   This class implements continuous-time LPV dynamics. It is intended 
    %   to model open or closed-loop agent dynamics. The dynamics must be
    %   given in the form of
    %
    %   xdot(t) = A(rho)*x(t) + B(rho)*w(t)
    %   z(t)    = C(rho)*x(t) + D(rho)*w(t),
    %
    %   where the output equation can be neglected. rho(t, x(t), w(t)) is
    %   the (vector-valued) scheduling parameter of the system.
    
    properties(GetAccess = public, SetAccess = protected)
        x % Dynamic state of the system
        t % Current time of the dynamic system
    end
    
    properties(GetAccess = public, SetAccess = immutable)
        % Sampling time of the overlaying discrete-time system. calling the
        % step function will simulate the continuous-time system in dT time
        % intervals.
        dT 
        
        % In the case of LPV systems, A, B, C, D are not matrices, but
        % matrix valued functions as defined in the comment above.
        A
        B
        C
        D
        
        % Function that maps to the scheduling parameter of the system
        rho_fun
    end
    
    methods
        function obj = ContinuousLpvDynamics(dT, rho, A, B, C, D, x0)
            %CONTINUOUSLPVDYNAMICS Construct an instance of this class
            %   Sets up the internal model for simulation of a
            %   continuous-time LPV model.
            %
            %   If no output is required, C & D can be set to []. If no
            %   initial state x0 is specified, x0 = 0 is used. The initial
            %   state must be passed in, as its dimension cannot be
            %   inferred from the state equation.
            
            obj.rho_fun = rho;
            obj.A  = makeHandle(A);
            obj.B  = makeHandle(B);
            
            obj.x  = x0;
            obj.t  = 0;
            obj.dT = dT;
            
            % If no output is required, you can either leave C & D out, or
            % pass in [] for both. This will in that case fix the
            % dimensions. Dropping only one will not work.
            if isempty(C) && isempty(D)
                obj.C = @(~) double.empty(0, length(x0));
                obj.D = @(~) double.empty(0, 1);
            else
                obj.C = makeHandle(C);
                obj.D = makeHandle(D);
            end
        end
        
        function z = step(obj, w)
            %STEP Method to execute a single time step of the dynamics.
            %   This function takes an input w and evaluates the state and
            %   output equations of the LPV system. Note that z contains
            %   the output that was valid at the beginning of the step.
            %
            %   The returned valued is empty if no C and D functions are
            %   specified.

            if nargout >= 1
                rho = obj.rho_fun(obj.t, obj.x, w);
                z   = obj.C(rho) * obj.x + obj.D(rho) * w;
            end
            
            % Solve ODE for one time step
            fun   = @(t,y) obj.A(obj.rho_fun(t,y,w)) * y ...
                         + obj.B(obj.rho_fun(t,y,w)) * w;
            [~,y] = ode45(fun, obj.t + [0, obj.dT], obj.x);
            obj.x = y(end, :)';
            
            % Advance system time
            obj.t = obj.t + obj.dT;
        end
    end
end