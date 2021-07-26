% This file is part of a library for simulation of multi-agent systems
% developed at the Institute of Control Systems at TUHH.
%
% Original Authors: Christian Hespe <christian.hespe@tuhh.de>

classdef IntegratorAgent < DynamicAgent
    %INTEGRATORAGENT Class that implements linear single integrator 
    %dynamics that are required e.g. for consensus protocols.
    
    properties(GetAccess = public, SetAccess = immutable)
        dim      % Number of independent integrators
    end
    
    % These properties have to be redefined from the superclass BaseAgent
    properties(Dependent, GetAccess = public, SetAccess = private)
        position % Current position of the agent
        velocity % Current velocity of the agent
    end
    
    methods
        function obj = IntegratorAgent(id, dT, initialPos, method)
            %INTEGRATORAGENT Construct an instance of this class
            %   Sets up the correct agent dynamics and initializes the
            %   agent with the correct initial position.
            %
            %   id          Id of the agent in the network
            %   dT          Desired sampling time
            %   initialPos  Initial position of the agent
            %   method      Method used for discretization, either 'zoh' or
            %               'euler', default: 'zoh'
            
            dim = length(initialPos);
            
            % Define continuous-time state-space matrices of appropriate
            % dimension.
            A = zeros(dim);
            B = eye(dim);
            
            if nargin <= 3
                method = 'zoh';
            end
            [Ad, Bd] = ltiDiscretization(A, B, dT, method);
              
            % Construct discrete-time state space model
            dynamics = DiscreteLtiDynamics(Ad, Bd, [], [], initialPos);
            
            obj@DynamicAgent(id, dT, dynamics);
            obj.dim = dim;
        end
        
        function value = get.position(obj)
            %GET.POSITION Implementation of the dependent position
            %property.
            %   This agent only as a position as its state space, thus the
            %   position is identical to the state.
            
            value = obj.state;
        end
        
        function value = get.velocity(obj)
            %GET.VELOCITY Implementation of the dependent velocity
            %property.
            %   The speed of this agent is at the same time its input, as
            %   it does not have any mass.
            
            value = obj.u;
        end
    end
end

