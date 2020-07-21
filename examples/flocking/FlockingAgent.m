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
        da      = 7;   % Equilibrium distance(or desired distance) to neighbours
        ra      = 1.2 * FlockingAgent.da; % Sensing radius (no interaction with agents outside ra disc)
        h       = 0.9; % Bump function goes to zero after h
        c_damp  = 3;   % Damping between agents / alignment rule
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
            obj.move(u);
            
            % Send message to network, include position and velocity
            data = struct;
            data.position = obj.position;
            data.velocity = obj.velocity;
            obj.send(data)
        end
    end
end

