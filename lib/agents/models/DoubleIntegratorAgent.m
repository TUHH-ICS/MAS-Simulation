% This file is part of a library for simulation of multi-agent systems
% developed at the Institute of Control Systems at TUHH.
%
% Original Authors: Christian Hespe <christian.hespe@tuhh.de>

classdef DoubleIntegratorAgent < DynamicAgent
    %DOUBLEINTEGRATORAGENT Class that implements double integrator dynamics
    %that are required e.g. for flocking protocols.
    
    properties(GetAccess = public, SetAccess = immutable)
        dim      % Number of independent integrators
    end
    
    % These properties have to be redefined from the superclass BaseAgent
    properties(Dependent, GetAccess = public, SetAccess = private)
        position % Current position of the agent
        velocity % Current velocity of the agent
    end
    
    methods
        function obj = DoubleIntegratorAgent(id, dT, initialPos, initialVel)
            %DOUBLEINTEGRATORAGENT Construct an instance of this class
            %   Sets up the correct agent dynamics and initializes the
            %   agent with the correct initial position and velocity.
            
            dim = length(initialPos);
            if dim ~= length(initialVel)
                error('Position and velocity vectors must have the same dimensions!')
            end
            
            % Define continuous-time state-space matrices. The matrices get
            % kroneckered up to a suitable dimension for the initial
            % conditions.
            A = kron(eye(dim), [0 1; 0 0]);
            B = kron(eye(dim), [0; 1]);
            [Ad, Bd] = ltiDiscretization(A, B, dT);
              
            % Construct discrete-time state space model
            x0 = kron(initialPos, [1; 0]) + kron(initialVel, [0; 1]);
            dynamics = DiscreteLtiDynamics(Ad, Bd, [], [], x0);
            
            obj@DynamicAgent(id, dT, dynamics);
            obj.dim = dim;
        end
        
        function value = get.position(obj)
            %GET.POSITION Implementation of the dependent position
            %property.
            %   This function is simply a projection from the state space 
            %   of the agents into the position space.
            
            value = obj.state(2*(1:obj.dim) - 1);
        end
        
        function value = get.velocity(obj)
            %GET.VELOCITY Implementation of the dependent velocity
            %property.
            %   This function is simply a projection from the state space
            %   of the agents into the velocity space.
            
            value = obj.state(2*(1:obj.dim));
        end
    end
end

