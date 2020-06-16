classdef DiscreteNonlinearDynamics < DiscreteDynamics
    %DISCRETENONLINEARDYNAMICS Convenience class for the simulation of
    %general discrete-time nonlinear dynamics.
    %   This class implements general nonlinear dynamics in state-space
    %   form. The dynamics must be given in the form of
    %
    %   x(k+1) = f(k, x(k), w(k))
    %   z(k)   = h(k, x(k), w(k)),
    %
    %   where the output equation can be neglected, and can thus be
    %   nonlinear, time-dependent and biprober.
    
    % These two functions are used to define the system dynamics as stated
    % in the comment above.
    properties(GetAccess = public, SetAccess = immutable)
        f
        h
    end
    
    methods
        function obj = DiscreteNonlinearDynamics(f, h, x0)
            %DISCRETENONLINEARDYNAMICS Construct an instance of this class
            %   Sets up the internal model for simulation of a
            %   discrete-time nonlinear model.
            %
            %   If no output is required, h can be set to []. If no initial
            %   state x0 is specified, x0 = 0 is used.
            
            % Initialize state to default value, if no value is given
            if nargin <= 2
                vek = ProbingValue.withDimension([], 1);
                sz  = size(f(0, vek, vek));
                
                if any(isnan(sz))
                    x0 = 0;
                else
                    x0 = zeros(sz);
                end
            end
            
            obj@DiscreteDynamics(x0);
            obj.f = f;
            
            % If no output is required, this will set a default
            if nargin <= 1 || isempty(h)
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
                z = obj.h(obj.k, obj.x, w);
            end
            
            obj.x = obj.f(obj.k, obj.x, w);
            
            % Advance system time
            obj.k = obj.k + 1;
        end
    end
end

