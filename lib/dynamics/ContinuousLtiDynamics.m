classdef ContinuousLtiDynamics < ContinuousDynamics
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
    
    properties(GetAccess = public, SetAccess = immutable)       
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
            
            % Initialize state to default value, if no value is given
            if nargin <= 5
                x0 = zeros(size(A,1), 1);
            end
            
            obj@ContinuousDynamics(dT, x0);
            obj.A  = A;
            obj.B  = B;
            
            % If no output is required, you can either leave C & D out, or
            % pass in [] for both. This will in that case fix the
            % dimensions. Dropping only one will not work.
            if nargin <= 3 || (isempty(C) && isempty(D))
                obj.C = double.empty(0, size(A,2));
                obj.D = double.empty(0, size(B,2));
            else
                obj.C = C;
                obj.D = D;
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
            
            % Advance system time
            obj.t = obj.t + obj.dT;
        end
    end
end