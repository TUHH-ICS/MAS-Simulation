classdef FlockingAgent < BaseAgent
    %FLOCKINGAGENT Examplary implementation of double integrator agents
    %that perform a flocking manoeuvre.
    
    % Define constants of the flocking protocol
    properties(Constant)
        epsilon = 0.1; % Used to define the sigma_norm
        da      = 7;   % Equilibrium distance(or desired distance) to neighbours
        ra      = 1.2 * FlockingAgent.da; % Sensing radius (no interaction with agents outside ra disc)
        h       = 0.9; % Bump function goes to zero after h
        c_damp  = 3;   % Damping between agents / alignment rule
    end
    
    properties(Dependent, GetAccess = public, SetAccess = private)
        position % Current position of the agent
        velocity % Current velocity of the agent
    end
    
    methods     
        function obj = FlockingAgent(network, initialPos, initialVel)
            %FLOCKINGAGENT Construct an instance of this class
            %   Detailed explanation goes here
            
            x0 = kron(initialPos, [1; 0]) + kron(initialVel, [0; 1]);
            obj@BaseAgent(network, x0);
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
    end
    
    methods(Access = protected)
        function step(obj)
            % TODO The step size should not be a local constant in the
            % agent implementation, but set centrally in the simulation
            % framework. This needs some form of design work, because we
            % want to support time triggered and event triggered control.
            T = 0.1;
            
            % Receive messages from the network
            messages = obj.receive();
            
            % Implement the flocking protocol
            u = zeros(2, 1);
            for message = messages
                % Compute sigma distance between the two agents
                [dist, grad] = sigma_norm(message.data.position - obj.position,...
                                          FlockingAgent.epsilon);
                              
                % Calculate force on the agent from the virtual potential field
                u = u + grad * phi_alpha(dist, FlockingAgent.ra,...
                                         FlockingAgent.da, FlockingAgent.h);
                                     
                % Compute position dependent adjacency element
                a = rho_h(dist / FlockingAgent.ra, FlockingAgent.h);
                
                % Calculate alignment force
                u = u + FlockingAgent.c_damp * a * (message.data.velocity - obj.velocity);
            end
            
            % Implement double integrator dynamics
            obj.x(1) = obj.x(1) + T*obj.x(2);
            obj.x(2) = obj.x(2) + T*u(1);
            obj.x(3) = obj.x(3) + T*obj.x(4);
            obj.x(4) = obj.x(4) + T*u(2);
            
            % Send message to network, include position and velocity
            data = struct;
            data.position = obj.position;
            data.velocity = obj.velocity;
            obj.send(data)
        end
    end
end

