classdef ContinuousNonlinearDynamics < ContinuousDynamics
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
    
    properties(GetAccess = public, SetAccess = immutable)       
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
            %   If no output is required, h can be set to []. If no initial
            %   state x0 is specified, x0 = 0 is used.
            
            % Initialize state to default value, if no value is given
            if nargin <= 3
                vek = ProbingValue.withDimension([], 1);
                sz  = size(f(0, vek, vek));
                
                if any(isnan(sz))
                    x0 = 0;
                else
                    x0 = zeros(sz);
                end
            end
            
            obj@ContinuousDynamics(dT, x0);
            obj.f  = f;
            
            % If no output is required, this will fix set a default
            if nargin <= 2 || isempty(h)
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