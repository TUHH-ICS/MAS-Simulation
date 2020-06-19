classdef FlockingAgent2 < BaseAgent
    %FLOCKINGAGENT2 Examplary implementation of double integrator agents
    %that perform a flocking manoeuvre augmented with a source seeking
    % forcing term.
    
    % Define constants of the flocking protocol
    properties(Constant)        
        c_damp          = 3;   % Damping between agents / alignment rule
        c_self_damp     = 1;   % Damping between agents and environment (e.g Viscocity)
        c_gradient      = 1; % constant multiplying the gradient force
    end
    
    % This agent implementation chooses not to use a predefined dynamics
    % object, so it has to declare the state on its own.
    properties(Access = private)
        x        % Local variable for the agent state
    end
    
    properties(GetAccess = private, SetAccess = immutable)
        interac_field % Interaction field with other agents
        conc_field  % Pointer to the network object, for sending and receiving
    end
    
    % These properties have to be redefined from the superclass BaseAgent
    properties(Dependent, GetAccess = public, SetAccess = private)
        position % Current position of the agent
        velocity % Current velocity of the agent
        state    % Dynamic state of the agent
    end
    
    methods     
        function obj = FlockingAgent2(id, dT, initialPos, initialVel, conc_field,interac_field)
            %FLOCKINGAGENT Construct an instance of this class
            %   Initializes the state space to the given initial position
            %   and velocity.
            obj@BaseAgent(id, dT);
            obj.x     = kron(initialPos, [1; 0]) + kron(initialVel, [0; 1]);
			obj.conc_field=conc_field;
            obj.interac_field=interac_field;
        end        
        
        function value = get.state(obj)
            %GET.STATE Implementation of the dependent state property.
            value = obj.x;
        end
        
        function value = get.position(obj)
            %GET.POSITION Implementation of the dependent position
            %property.
            %   This function is simply a projection from the state space 
            %   of the agents into the position space.
            value = [obj.x(1); obj.x(3)];
        end
        
        function value = get.velocity(obj)
            %GET.VELOCITY Implementation of the dependent velocity
            %property.
            %   This function is simply a projection from the state space
            %   of the agents into the velocity space.
            value = [obj.x(2); obj.x(4)];
        end
        
        function grad_est = get_gradient_est(obj)
            %get_gradient_est
            % This function estimates the gradient of the field at the
            % current agent location. In the current implementation, we
            % just perturb the true gradient by noise. 
            % 
            % Next step should be to querry a data set (say 10 points) near
            % the current agent location and solve a LS problem to estimate
            % the gradient.
            grad=obj.conc_field.get_true_gradient(obj.position);            
            grad_est=grad+norm(grad)*0.1*(-0.5+rand(size(grad,1),1));
        end
        
        
        function step(obj)
            % Receive messages from the network
            messages = obj.receive();
            
            % Implement the flocking protocol
            u = zeros(2, 1);
            for message = messages                                
                % Force due to interaction field potential field and the
                % adjacency element betwen agents
                [force,a]=obj.interac_field.get_interaction_force(message.data.position - obj.position);
                u = u + force;                     
                % Calculate alignment force
                u = u + FlockingAgent2.c_damp * a * (message.data.velocity - obj.velocity);
            end
            
            % Calculate force on the agent from the source field            
            u = u -  obj.c_gradient*get_gradient_est(obj); 
            % Self_damping
            u = u - FlockingAgent2.c_self_damp*obj.velocity; 
                        
            % Implement double integrator dynamics
            obj.x(1) = obj.x(1) + obj.dT * obj.x(2);
            obj.x(2) = obj.x(2) + obj.dT * u(1);
            obj.x(3) = obj.x(3) + obj.dT * obj.x(4);
            obj.x(4) = obj.x(4) + obj.dT * u(2);
            
            % Send message to network, include position and velocity
            data = struct;
            data.position = obj.position;
            data.velocity = obj.velocity;
            obj.send(data)
        end
    end
end

