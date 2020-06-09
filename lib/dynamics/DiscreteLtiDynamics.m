classdef DiscreteLtiDynamics < handle
    %DISCRETELTIDYNAMICS Convenience class for the simulation of LTI
    %systems
    %   This class implements discrete-time LTI dynamics. It is intended to
    %   model open or closed-loop agent dynamics. 
    
    properties(GetAccess = public, SetAccess = protected)
        x % Dynamic state of the system
    end
    
    % These matrices are the usual discrete-time matrices that represent a
    % state-space model of an LTI system.
    properties(GetAccess = public, SetAccess = immutable)
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
        function obj = DiscreteLtiDynamics(A, B, C, D, x0)
            %DISCRETELTIDYNAMICS Construct an instance of this class
            %   Sets up the internal model for simulation of a
            %   discrete-time LTI model.
            
            obj.A = A;
            obj.B = B;
            obj.C = C;
            obj.D = D;
            
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
            if nargin <= 4
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
            obj.x = obj.A * obj.x + obj.B * w;
        end
    end
end

