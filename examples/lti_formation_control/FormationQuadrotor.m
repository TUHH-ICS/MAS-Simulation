classdef FormationQuadrotor < BaseAgent
    %FORMATIONQUADROTOR Quadrotor agent that performs a formation control
    %maneuvre in a group of agents.
    %   The agent dynamics are approximated as a LTI system using Jacobian
    %   linearization. See ATC exercise 6.3).
    
    properties(GetAccess = public, SetAccess = immutable)
        ref      % Formation reference
    end
    
    properties(GetAccess = private, SetAccess = immutable)
        dynamics % Discrete-time LTI agent dynamics
    end
    
    % These properties have to be redefined from the superclass BaseAgent
    properties(Dependent, GetAccess = public, SetAccess = private)
        position % Current position of the agent
        velocity % Current velocity of the agent
        state    % Dynamic state of the agent
    end
    
    methods
        function obj = FormationQuadrotor(network, dT, initialPos, reference)
            %FORMATIONQUADROTOR Construct an instance of this class
            %   Sets up the correct agent dynamics and initializes the
            %   agent and consensus protocol to the given initial position.
            
            obj@BaseAgent(network, dT);
            
            % Import quadrotor model and controller
            data = load('quadrotor_model');
            
            % Build closed-loop dynamic system
            CL   = lft(data.Paug*data.K, -eye(length(eig(data.Paug))));
            x0 = [ kron(initialPos, [1; 0]); zeros(25,1) ];
            
            discrete = true;
            if discrete
                dCL  = c2d(CL, dT);
                [A,B] = ssdata(dCL);
                obj.dynamics = DiscreteLtiDynamics(A, B, [], [], x0);
            else
                [A,B] = ssdata(CL);
                obj.dynamics = ContinuousLtiDynamics(dT, A, B, [], [], x0);
            end
                
            % Set the formation reference if one is given to the agent
            if nargin <= 3
                obj.ref = zeros(size(initialPos));
            else
                obj.ref = reference;
            end
        end
        
        function value = get.state(obj)
            %GET.STATE Implementation of the dependent state property.
            value = obj.dynamics.x;
        end
        
        function value = get.position(obj)
            %GET.POSITION Implementation of the dependent position
            %property.
            %   This function is simply a projection from the state space 
            %   of the agents into the position space.
            value = [obj.dynamics.x(1); obj.dynamics.x(3); obj.dynamics.x(5)];
        end
        
        function value = get.velocity(obj)
            %GET.VELOCITY Implementation of the dependent velocity
            %property.
            %   This function is simply a projection from the state space
            %   of the agents into the velocity space.
            value = [obj.dynamics.x(2); obj.dynamics.x(4); obj.dynamics.x(6)];
        end
        
        function step(obj)
            % Receive messages from the network
            messages = obj.receive();
            
            if ~isempty(messages)
                % Calculate new formation reference. We use the normalized
                % Laplacian, therefore we calculate the mean of the
                % positions, not the sum
                data        = [messages.data];
                positions   = [data.position];
                dist        = mean(positions, 2) + obj.ref - obj.position;
            else
                dist        = zeros(3,1);
            end
            
            % Evaluate agent dynamics
            obj.dynamics.step(dist);
            
            % Send message to network, include only the position 
            data = struct;
            data.position = obj.position - obj.ref;
            obj.send(data)
            
            % Execute BaseAgent step
            obj.step@BaseAgent()
        end
    end
end

