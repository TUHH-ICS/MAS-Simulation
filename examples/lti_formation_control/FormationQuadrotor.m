classdef FormationQuadrotor < DynamicAgent
    %FORMATIONQUADROTOR Quadrotor agent that performs a formation control
    %maneuvre in a group of agents.
    %   The agent dynamics are approximated as a LTI system using Jacobian
    %   linearization. See ATC exercise 6.3).
    
    properties(GetAccess = public, SetAccess = immutable)
        ref      % Formation reference
    end
    
    % These properties have to be redefined from the superclass BaseAgent
    properties(Dependent, GetAccess = public, SetAccess = private)
        position % Current position of the agent
        velocity % Current velocity of the agent
    end
    
    methods
        function obj = FormationQuadrotor(id, dT, initialPos, reference)
            %FORMATIONQUADROTOR Construct an instance of this class
            %   Sets up the correct agent dynamics and initializes the
            %   agent and consensus protocol to the given initial position.
            
            % Import quadrotor model and controller
            data = load('quadrotor_model');
            
            % Build closed-loop dynamic system
            CL   = lft(data.Paug*data.K, -eye(length(eig(data.Paug))));
            x0 = [ kron(initialPos, [1; 0]); zeros(25,1) ];
            
            discrete = true;
            if discrete
                dCL  = c2d(CL, dT);
                [A,B] = ssdata(dCL);
                dynamics = DiscreteLtiDynamics(A, B, [], [], x0);
            else
                [A,B] = ssdata(CL);
                dynamics = ContinuousLtiDynamics(dT, A, B, [], [], x0);
            end
                  
            obj@DynamicAgent(id, dT, dynamics);
            
            % Set the formation reference if one is given to the agent
            if nargin <= 3
                obj.ref = zeros(size(initialPos));
            else
                obj.ref = reference;
            end
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
    end
       
    methods(Access = protected)
        function control(obj)
            % Receive messages from the network
            messages = obj.receive();
            
            if ~isempty(messages)
                % Calculate new formation reference. We use the normalized
                % Laplacian, therefore we calculate the mean of the
                % positions, not the sum
                data      = [messages.data];
                positions = [data.position];
                obj.u     = mean(positions, 2) + obj.ref - obj.position;
            else
                obj.u     = zeros(3,1);
            end
            
            % Send message to network, include only the position 
            data = struct;
            data.position = obj.position - obj.ref;
            obj.send(data)
        end
    end
end

