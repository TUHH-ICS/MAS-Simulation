classdef FormationAgent < DynamicAgent
    %FORMATIONAGENT Examplary agent implementation with simple LTI dynamics
    %that performs a formation control manoeuvre.
    
    % Define the constants of the agent and the consensus protocol
    properties(Constant)
        b       = 1     % Friction coefficient
        m       = 1     % Mass of the agent
        epsilon = 0.01; % Convergence speed of the consensus protocol
    end
    
    properties(GetAccess = public, SetAccess = protected)
        consens  % State of the consensus protocol
    end
    
    properties(GetAccess = public, SetAccess = immutable)
        Fx       % State feedback coefficient
        Fr       % Feed-forward coefficient
        ref      % Formation reference
    end
    
    % These properties have to be redefined from the superclass BaseAgent
    properties(Dependent, GetAccess = public, SetAccess = private)
        position % Current position of the agent
        velocity % Current velocity of the agent
    end
    
    methods
        function obj = FormationAgent(id, dT, initialPos, reference)
            %FORMATIONAGENT Construct an instance of this class
            %   Sets up the correct agent dynamics and initializes the
            %   agent and consensus protocol to the given initial position.
            
            % Define the mass friction system
            A = [ 1  dT                                       ;
                  0  1 - dT*FormationAgent.b/FormationAgent.m ];
            B = [ 0; dT/FormationAgent.m ];
            
            % Kronecker up to 2 dimensions
            A_bar = kron(eye(3), A);
            B_bar = kron(eye(3), B);
            x0    = kron(initialPos, [1; 0]);
            
            % Initialize agent dynamics
            dynamics = DiscreteLtiDynamics(A_bar, B_bar, [], [], x0);
            obj@DynamicAgent(id, dT, dynamics);
            
            % Initialize the consensus protocol to the initial position of
            % the agent itself
            obj.consens = initialPos;
            
            % Set the formation reference if one is given to the agent
            if nargin <= 3
                obj.ref = zeros(size(initialPos));
            else
                obj.ref = reference;
            end
            
            % Set controller matrices
            obj.Fr = kron(eye(3),   168.8665);
            obj.Fx = kron(eye(3), [-168.8668, -25.0094]);
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
                data        = [messages.data];
                positions   = [data.position];
                dist        = mean(obj.position - positions - obj.ref, 2);
                obj.consens = obj.consens - obj.epsilon * dist;
            end
            
            % Evaluate controller equation
            obj.u = obj.Fr * obj.consens + obj.Fx * obj.dynamics.x;
            
            % Send message to network, include only the position 
            data = struct;
            data.position = obj.position - obj.ref;
            obj.send(data)
        end
    end
end

