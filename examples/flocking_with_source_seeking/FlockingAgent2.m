classdef FlockingAgent2 < BaseAgent
    %FLOCKINGAGENT2 Examplary implementation of double integrator agents
    %that perform a flocking manoeuvre augmented with a source seeking
    % forcing term.
    
    % Define constants of the flocking protocol
    properties(Constant)
        epsilon         = 0.1; % Used to define the sigma_norm
        da              = 7;   % Equilibrium distance(or desired distance) to neighbours
        ra              = 1.2 * FlockingAgent2.da; % Sensing radius (no interaction with agents outside ra disc)
        h               = 0.9; % Bump function goes to zero after h
        c_damp          = 3;   % Damping between agents / alignment rule
        c_self_damp     = 1;   % Damping between agents and environment (e.g Viscocity)
        c_gradient      = 1; % constant multiplying the gradient force
    end
    
    properties(Dependent, GetAccess = public, SetAccess = private)
        position % Current position of the agent
        velocity % Current velocity of the agent
    end
    properties(GetAccess = public, SetAccess = protected)       
        gradient % gradient of the field at the position of agent
    end
    properties(GetAccess = private, SetAccess = immutable)
        field  % Pointer to the network object, for sending and receiving
    end
    
    methods     
        function obj = FlockingAgent2(network, dT, initialPos, initialVel, field)
            %FLOCKINGAGENT Construct an instance of this class
            %   Initializes the state space to the given initial position
            %   and velocity.
            
            x0 = kron(initialPos, [1; 0]) + kron(initialVel, [0; 1]);
            obj@BaseAgent(network, dT, x0);
            obj.field=field;
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
        
        function [] = get_gradient_est(obj)
            %get_gradient_est
            % This function estimates the gradient of the field at the
            % current agent location. In the current implementation, we
            % just perturb the true gradient by noise. 
            % 
            % Next step should be to querry a data set (say 10 points) near
            % the current agent location and solve a LS problem to estimate
            % the gradient.
            grad=obj.field.get_true_gradient(obj.position);            
            obj.gradient=grad+norm(grad)*0.1*(-0.5+rand(size(grad,1),1));
        end
        
        
        function step(obj)
            % Receive messages from the network
            messages = obj.receive();
            
            % Implement the flocking protocol
            u = zeros(2, 1);
            for message = messages
                % Compute sigma distance between the two agents
                [dist, grad] = sigma_norm(message.data.position - obj.position,...
                                          FlockingAgent2.epsilon);
                              
                % Calculate force on the agent from the virtual potential field
                u = u + grad * phi_alpha(dist, FlockingAgent2.ra,...
                                         FlockingAgent2.da, FlockingAgent2.h);
                                     
                % Compute position dependent adjacency element
                a = rho_h(dist / FlockingAgent2.ra, FlockingAgent2.h);               
                              
                
                % Calculate alignment force
                u = u + FlockingAgent2.c_damp * a * (message.data.velocity - obj.velocity);
            end
            
            % Calculate force on the agent from the source field
            obj.get_gradient_est();
            u = u -  obj.c_gradient*obj.gradient; 
            
            
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
            
            % Execute BaseAgent step
            obj.step@BaseAgent()
        end
    end
end

