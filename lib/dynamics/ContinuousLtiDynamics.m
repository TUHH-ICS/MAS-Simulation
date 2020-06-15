classdef ContinuousLtiDynamics < handle
    %CONTINUOSLTIDYNAMICS Convenience class for the simulation of LTI
    %systems
    %   This class implements continuous-time LTI dynamics. It is intended
    %   to model open or closed-loop agent dynamics. The dynamics must be
    %   given in the form of
    %
    %   xdot(t) = A*x(t) + B*w(t)
    %   z(t)    = C*x(t) + D*w(t),
    %
    %   where the output equation can be neglected.
    
    properties(GetAccess = public, SetAccess = protected)
        x % Dynamic state of the system
    end
    
    properties(GetAccess = public, SetAccess = immutable)
        % Sampling time of the overlaying discrete-time system. calling the
        % step function will simulate the continuous-time system in dT time
        % intervals.
        dT 
        
        % These matrices are the continuous-time system matrices that are
        % defined in the comment above.
        A
        B
        C
        D
    end

    methods
        function obj = ContinuousLtiDynamics(dT, A, B, C, D, x0)
            %CONTINUOUSLTIDYNAMICS Construct an instance of this class
            %   Sets up the interal model for simulation of a
            %   continuous-time LTI model.
            %
            %   If no output is required, C & D can be set to []. If no
            %   initial state x0 is specified, x0 = 0 is used.
            
            obj.A  = A;
            obj.B  = B;
            
            obj.dT = dT;
            
            % If no output is required, you can either leave C & D out, or
            % pass in [] for both. This will in that case fix the
            % dimensions. Dropping only one will not work.
            if (nargin <= 3) || (isempty(C) && isempty(D))
                obj.C = double.empty(0, size(A,2));
                obj.D = double.empty(0, size(B,2));
            else
                obj.C = C;
                obj.D = D;
            end
            
            % Initialize state to default value, if no value is given
            if nargin <= 5
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
            
            if nargout >= 1
                z = obj.C * obj.x + obj.D * w;
            end

            % Solve ODE for one time step
            fun   = @(~,y) obj.A * y + obj.B * w;
            [~,y] = ode45(fun, [0, obj.dT], obj.x);
            obj.x = y(end, :)';
        end
    end
end