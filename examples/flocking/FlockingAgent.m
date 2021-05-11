% This file is part of a library for simulation of multi-agent systems
% developed at the Institute of Control Systems at TUHH.
%
% Original Authors: Christian Hespe <christian.hespe@tuhh.de>

classdef FlockingAgent < DoubleIntegratorAgent
    %FLOCKINGAGENT Examplary implementation of double integrator agents
    %that perform a flocking manoeuvre.
    
    % Define constants of the flocking protocol
    properties(Constant)
        epsilon = 0.1; % Used to define the sigma_norm
        d       = 7;   % Equilibrium distance(or desired distance) to neighbours
        r       = 1.2 * FlockingAgent.d; % Sensing radius (no interaction with agents outside ra disc)
        ha      = 0.9; % Bump function coefficient for alpha agents
                
        da      = sigma_norm(FlockingAgent.d, FlockingAgent.epsilon)
        ra      = sigma_norm(FlockingAgent.r, FlockingAgent.epsilon)
        
        c1_a    = 1;   % P coefficient for the other alpha agents
        c2_a    = 2*sqrt(FlockingAgent.c1_a); % PD coefficient for the other alpha agents
    end
    
    methods     
        function obj = FlockingAgent(id, dT, initialPos, initialVel)
            %FLOCKINGAGENT Construct an instance of this class
            %   Initializes the state space to the given initial position
            %   and velocity.
            
            obj@DoubleIntegratorAgent(id, dT, initialPos, initialVel);
        end

        function step(obj)
            % Receive messages from the network
            messages = obj.receive();
            
            % Implement the flocking protocol
            u = zeros(obj.dim, 1);
            for message = messages
                % Compute sigma distance between the two agents
                [dist, grad] = sigma_norm(message.data.position - obj.position,...
                                          FlockingAgent.epsilon);
                              
                % Calculate force on the agent from the virtual potential field
                u = u + FlockingAgent.c1_a...
                       * grad * phi_alpha(dist, FlockingAgent.ra,...
                                          FlockingAgent.da, FlockingAgent.ha);
                                     
                % Compute position dependent adjacency element
                a = rho_h(dist / FlockingAgent.ra, FlockingAgent.ha);
                
                % Calculate alignment force
                u = u + FlockingAgent.c2_a * a * (message.data.velocity - obj.velocity);
            end
            
            % Evaluate double integrator dynamics
            obj.move(u);
            
            % Send message to network, include position and velocity
            data = struct;
            data.position = obj.position;
            data.velocity = obj.velocity;
            obj.send(data)
        end
    end
end

