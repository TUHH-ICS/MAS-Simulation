classdef ContinuousLtiDynamics < handle
    %CONTINUOSLTIDYNAMICS Convenience class for the simulation of LTI
    %systems
    %   This class implements continuous-time LTI dynamics. It is intended
    %   to model open or closed-loop agent dynamics. 
    
    properties(GetAccess = public, SetAccess = protected)
        x % Dynamic state of the system
    end
    
    properties(GetAccess = public, SetAccess = immutable)
        % Sampling time of the overlaying discrete-time system. calling the
        % step function will simulate the continuous time system in dT time
        % intervals.
        dT 
        
        % These matrices are the usual continuous-time matrices that
        % represent a state-space model of an LTI system.
        A
        B
        C
        D
    end
    
    properties(Access = protected)
        w % Last input to the system. Gets saved to later calculate z
    end
    
    properties(Dependent, GetAccess = public, SetAccess = private)
        z % Output of the system
    end
    
    methods
        function obj = ContinuousLtiDynamics(A, B, C, D, dT, x0)
            %CONTINUOUSLTIDYNAMICS Construct an instance of this class
            %   Sets up the interal model for simulation of a
            %   continuous-time LTI model.
            
            obj.dT = dT;
            obj.A  = A;
            obj.B  = B;
            obj.C  = C;
            obj.D  = D;
            
            % If no output is required, you can simply pass in [] and this
            % will fix the dimensions.
            if isempty(C) && isempty(D)
                obj.C = double.empty(0, size(A,2));
                obj.D = double.empty(0, size(B,2));
            end
            
            % Start with NaN as last input. In this way, you cannot forget
            % to first call the step function, before polling the output.
            obj.w = NaN;
            
            % Initialize state to default value, if no value is given
            if nargin <= 5
                obj.x = zeros(size(A,1), 1);
            else
                obj.x = x0;
            end
        end
        
        function value = get.z(obj)
            %GET.Z Implementation of the dependent z property.
            %   Evaluates the output equation of this LTI system. As this
            %   call is made indepentently of a call to step(w), the last
            %   value of the input needs to be saved.
            %   The returned valued may be empty if no C and D matrices a
            %   specified.
            value = obj.C * obj.x + obj.D * obj.w;
        end
        
        function step(obj, w)
            %STEP Method to execute a single time step of the dynamics.
            %   This function takes an input w and evaluates the state
            %   equation of the LTI system.
            obj.w = w;
            
            % Set up ODE function with current input
            fun   = @(t, y) obj.A * y + obj.B * w;
            
            % Solve ODE for one time step
            [~,y] = ode45(fun, [0, obj.dT], obj.x);
            obj.x = y(end, :)';
        end
    end
end

