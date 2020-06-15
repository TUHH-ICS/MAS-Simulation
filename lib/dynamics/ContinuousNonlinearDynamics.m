classdef ContinuousNonlinearDynamics < handle
    %CONTINUOUSNONLINEARDYNAMICS Convenience class for the simulation of
    %general continuous-time nonlinear dynamics.
    %   This class implements general nonlinear dynamics in state-space
    %   form. The dynamics must be given in the form of
    %
    %   xdot(t) = f(t, x(t), w(t))
    %   z(t)    = h(t, x(t), w(t)),
    %
    %   where the output equation can be neglected, and can thus be
    %   nonlinear, time-dependent and biprober.
    
    properties(GetAccess = public, SetAccess = protected)
        x % Dynamic state of the system
        t % Current time of the dynamic system
    end
    
    properties(GetAccess = public, SetAccess = immutable)
        % Sampling time of the overlaying discrete-time system. calling the
        % step function will simulate the continuous-time system in dT time
        % intervals.
        dT 
        
        % These two functions are used to define the system dynamics as
        % stated in the comment above.
        f
        h
    end

    methods
        function obj = ContinuousNonlinearDynamics(dT, f, h, x0)
            %CONTINUOUSNONLINEARDYNAMICS Construct an instance of this
            %class
            %   Sets up the internal model for simulation of a
            %   continuous-time nonlinear model.
            %
            %   If no output is required, h can be set to []. The initial
            %   state must be passed in, as its dimension cannot be
            %   inferred from the state equation.
            
            obj.f  = f;
            
            obj.x  = x0;
            obj.t  = 0;
            obj.dT = dT;
            
            % If no output is required, this will fix set a default
            if isempty(h)
                obj.h = @(~,~,~)[];
            else
                obj.h = h;
            end          
        end
        
        function z = step(obj, w)
            %STEP Method to execute a single time step of the dynamics.
            %   This function takes an input w and evaluates the state and
            %   output equations of the nonlinear system. Note that z 
            %   contains the output that was valid at the beginning of the
            %   step.
            %
            %   The returned valued is empty if no output equation is
            %   specified.
            
            if nargout >= 1
            	z = obj.h(obj.t, obj.x, w);
            end
            
            % Solve ODE for one time step
            fun   = @(t,y) obj.f(t, y, w);
            [~,y] = ode45(fun, obj.t + [0, obj.dT], obj.x);
            obj.x = y(end, :)';
            
            % Advance system time
            obj.t = obj.t + obj.dT;
        end
    end
end