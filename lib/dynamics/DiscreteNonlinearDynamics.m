classdef DiscreteNonlinearDynamics < handle
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
    
    properties(GetAccess = public, SetAccess = protected)
        x % Dynamic state of the system
        k % Current timestep of the dynamic system
    end
    
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
            %   If no output is required, h can be set to []. The initial
            %   state must be passed in, as its dimension cannot be
            %   inferred from the state equation.
            
            obj.f = f;
            
            obj.x = x0;
            obj.k = 0;
            
            % If no output is required, this will set a default
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
                z = obj.h(obj.k, obj.x, w);
            end
            
            obj.x = obj.f(obj.k, obj.x, w);
            
            % Advance system time
            obj.k = obj.k + 1;
        end
    end
end

