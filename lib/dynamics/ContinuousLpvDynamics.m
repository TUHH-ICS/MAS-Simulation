% This file is part of a library for simulation of multi-agent systems
% developed at the Institute of Control Systems at TUHH.
%
% Original Authors: Christian Hespe <christian.hespe@tuhh.de>

classdef ContinuousLpvDynamics < ContinuousDynamics
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
    
    properties(GetAccess = public, SetAccess = immutable)      
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
            %   initial state x0 is specified, x0 = 0 is used.
            
            % Initialize state to default value, if no value is given
            if nargin <= 6
                if isa(A, 'function_handle')
                    vek = ProbingValue.withDimension([], 1);
                    sz  = size(A(vek));

                    if any(isnan(sz))
                        x0 = 0;
                    else
                        x0 = zeros(sz(1), 1);
                    end
                else
                    x0 = zeros(size(A,1), 1);
                end
            end
            
            obj@ContinuousDynamics(dT, x0);
            obj.rho_fun = rho;
            obj.A  = makeHandle(A);
            obj.B  = makeHandle(B);
            
            % If no output is required, you can either leave C & D out, or
            % pass in [] for both. This will in that case fix the
            % dimensions. Dropping only one will not work.
            if nargin <= 4 || (isempty(C) && isempty(D))
                if isa(B, 'function_handle')
                    vek = ProbingValue.withDimension([], 1);
                    sz  = size(B(vek), 2);

                    if isnan(sz)
                        error('Size of D cannot be determined automatically.')
                    end
                else
                    sz = size(B, 2);
                end
                
                obj.C = @(~) double.empty(0, length(x0));
                obj.D = @(~) double.empty(0, sz);
            else
                obj.C = makeHandle(C);
                obj.D = makeHandle(D);
            end
        end
        
        function z = step(obj, w, rho)
            %STEP Method to execute a single time step of the dynamics.
            %   This function takes an input w and evaluates the state and
            %   output equations of the LPV system. Note that z contains
            %   the output that was valid at the beginning of the step.
            %
            %   The returned valued is empty if no C and D functions are
            %   specified.

            if ~xor(isempty(obj.rho_fun), nargin <= 2)
                error('You must set either a rho_fun or give an external rho, not both!')
            end
            
            if ~isempty(obj.rho_fun)               
                fun = @obj.rho_fun;
            else
                fun = @(t, x, w) rho;
            end
            
            if nargout >= 1
                val = fun(obj.t, obj.x, w);
                z   = obj.C(val) * obj.x + obj.D(val) * w;
            end
            
            % Solve ODE for one time step
            odefun = @(t,y) obj.A(fun(t,y,w)) * y ...
                          + obj.B(fun(t,y,w)) * w;
            [~,y]  = ode45(odefun, obj.t + [0, obj.dT], obj.x);
            obj.x = y(end, :)';
            
            % Advance system time
            obj.t = obj.t + obj.dT;
        end
    end
end