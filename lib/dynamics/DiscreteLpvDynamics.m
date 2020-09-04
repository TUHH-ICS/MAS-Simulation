% This file is part of a library for simulation of multi-agent systems
% developed at the Institute of Control Systems at TUHH.
%
% Original Authors: Christian Hespe <christian.hespe@tuhh.de>

classdef DiscreteLpvDynamics < DiscreteDynamics
    %DISCRETELPVDYNAMICS Convenience class for the simulation of LPV
    %systems
    %   This class implements discrete-time LPV dynamics. It is intended to
    %   model open or closed-loop agent dynamics. The dynamics must be
    %   given in the form of
    %
    %   x(k+1) = A(rho)*x(k) + B(rho)*w(k)
    %   z(k)   = C(rho)*x(k) + D(rho)*w(k),
    %
    %   where the output equation can be neglected. rho(k, x(k), w(k)) is
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
        function obj = DiscreteLpvDynamics(rho, A, B, C, D, x0)
            %DISCRETELPVDYNAMICS Construct an instance of this class
            %   Sets up the internal model for simulation of a
            %   discrete-time LPV model.
            %
            %   If no output is required, C & D can be set to []. If no
            %   initial state x0 is specified, x0 = 0 is used.
            
            % Initialize state to default value, if no value is given
            if nargin <= 5
                if isa(A, 'function_handle')
                    vek = ProbingValue.withDimension([], 1);
                    sz  = size(A(vek));

                    if any(isnan(sz))
                        x0 = 0;
                    else
                        x0= zeros(sz(1), 1);
                    end
                else
                    x0 = zeros(size(A,1), 1);
                end
            end
            
            obj@DiscreteDynamics(x0);
            obj.rho_fun = rho;
            obj.A = makeHandle(A);
            obj.B = makeHandle(B);
            
            % If no output is required, you can either leave C & D out, or
            % pass in [] for both. This will in that case fix the
            % dimensions. Dropping only one will not work.
            if nargin <= 3 || (isempty(C) && isempty(D))
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
                rho = obj.rho_fun(obj.k, obj.x, w);
            end
            
            if nargout >= 1
                z = obj.C(rho) * obj.x + obj.D(rho) * w;    
            end
            
            obj.x = obj.A(rho) * obj.x + obj.B(rho) * w;
            
            % Advance system time
            obj.k = obj.k + 1;
        end
    end
end