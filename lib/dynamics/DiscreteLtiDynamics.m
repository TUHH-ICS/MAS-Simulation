classdef DiscreteLtiDynamics < handle
    %DISCRETELTIDYNAMICS Convenience class for the simulation of LTI
    %systems
    %   This class implements discrete-time LTI dynamics. It is intended to
    %   model open or closed-loop agent dynamics. The dynamics must be
    %   given in the form of
    %
    %   x(k+1) = A*x(k) + B*w(k)
    %   z(k)   = C*x(k) + D*w(k),
    %
    %   where the output equation can be neglected.
    
    properties(GetAccess = public, SetAccess = protected)
        x % Dynamic state of the system
    end
    
    % These matrices are the discrete-time system matrices as defined in
    % the comment above.
    properties(GetAccess = public, SetAccess = immutable)
        A
        B
        C
        D
    end
    
    methods
        function obj = DiscreteLtiDynamics(A, B, C, D, x0)
            %DISCRETELTIDYNAMICS Construct an instance of this class
            %   Sets up the internal model for simulation of a
            %   discrete-time LTI model.
            %
            %   If no output is required, C & D can be set to []. If no
            %   initial state x0 is specified, x0 = 0 is used.
            
            obj.A = A;
            obj.B = B;
            
            % If no output is required, you can either leave C & D out, or
            % pass in [] for both. This will in that case fix the
            % dimensions. Dropping only one will not work.
            if (nargin <= 2) || (isempty(C) && isempty(D))
                obj.C = double.empty(0, size(A,2));
                obj.D = double.empty(0, size(B,2));
            else
                obj.C = C;
                obj.D = D;
            end
            
            % Initialize state to default value, if no value is given
            if nargin <= 4
                obj.x = zeros(size(A,1), 1);
            else
                obj.x = x0;
            end
        end
        
        function z = step(obj, w)
            %STEP Method to execute a single time step of the dynamics.
            %   This function takes an input w and evaluates the state and
            %   output equations of the LTI system. Note that z contains
            %   the output that was valid at the beginning of the step.
            %
            %   The returned valued is empty if no C and D matrices are
            %   specified.
            
            z     = obj.C * obj.x + obj.D * w;
            obj.x = obj.A * obj.x + obj.B * w;
        end
    end
end