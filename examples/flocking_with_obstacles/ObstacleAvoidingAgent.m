% This file is part of a library for simulation of multi-agent systems
% developed at the Institute of Control Systems at TUHH.
%
% Original Authors: Christian Hespe <christian.hespe@tuhh.de>

classdef ObstacleAvoidingAgent < DoubleIntegratorAgent
    %OBSTACLEAVOIDINGAGENT Examplary implementation of a flocking protocol
    %with alpha, beta and gamma agents.
    %   This agent implements the flocking protocol proposed by
    %   Olfati-Saber, 2006, as Algorithm 3. It includes following a virtual
    %   leader agent, the gamma agent, and avoiding circular obstacles
    %   using virtual beta agents.
    
    % Define constants of the flocking protocol
    properties(Constant)
        epsilon = 0.1;  % Used to define the sigma_norm
        d       = 7;    % Equilibrium distance(or desired distance) to neighbours
        dprime  = 0.6 * ObstacleAvoidingAgent.d; % Desired distance to obstacles
        r       = 1.2 * ObstacleAvoidingAgent.d; % Sensing radius (no interaction with agents outside ra disc)
        
        ha      = 0.2;  % Bump function coefficient for alpha agents
        hb      = 0.9;  % Bump function coefficient for beta agents
        
        c1_a    = 1;   % P coefficient for the other alpha agents
        c1_b    = 4.5; % P coefficient for any obstacles
        c1_g    = 1.2; % P coefficient for feedback from gamma agent 

        da      = sigma_norm(ObstacleAvoidingAgent.d, ObstacleAvoidingAgent.epsilon)
        db      = sigma_norm(ObstacleAvoidingAgent.dprime, ObstacleAvoidingAgent.epsilon)
        ra      = sigma_norm(ObstacleAvoidingAgent.r, ObstacleAvoidingAgent.epsilon)

        c2_a    = 2*sqrt(ObstacleAvoidingAgent.c1_a); % PD coefficient for the other alpha agents
        c2_b    = 2*sqrt(ObstacleAvoidingAgent.c1_b); % PD coefficient for any obstacles
        c2_g    = 2*sqrt(ObstacleAvoidingAgent.c1_g); % PD coefficient for feedback from gamma agent
    end
    
    properties(GetAccess = public, SetAccess = private)
        q_gamma        % Position of the gamma agent
        p_gamma        % Velocity of the gamma agent
    end
    
    properties(GetAccess = public, SetAccess = immutable)
        obstacles      % list of sphere obstacles
    end
    
    methods     
        function obj = ObstacleAvoidingAgent(id, dT, initialPos, initialVel, obstacles)
            %OBSTACLEAVOIDINGAGENT Construct an instance of this class
            %   Initializes the state space to the given initial position
            %   and velocity.
            
            obj@DoubleIntegratorAgent(id, dT, initialPos, initialVel);
            
            obj.q_gamma = [ 60; 25 ];
            obj.p_gamma = [  4;  0 ];
            
            if nargin <= 4
                obj.obstacles = [];
            else
                obj.obstacles = obstacles;
            end
        end

        function step(obj)
            % Receive messages from the network
            messages = obj.receive();
            
            % Implement the alpha agent part of the flocking protocol
            u = zeros(2, 1);
            for message = messages
                % Compute sigma distance between the two agents
                [dist, grad] = sigma_norm(message.data.position - obj.position,...
                                          ObstacleAvoidingAgent.epsilon);
                              
                % Calculate force on the agent from the virtual potential field
                u = u + ObstacleAvoidingAgent.c1_a...
                        * grad * phi_alpha(dist, ObstacleAvoidingAgent.ra,...
                                           ObstacleAvoidingAgent.da, ObstacleAvoidingAgent.ha);
                                     
                % Compute position dependent adjacency element
                b = rho_h(dist / ObstacleAvoidingAgent.ra, ObstacleAvoidingAgent.ha);
                
                % Calculate alignment force
                u = u + ObstacleAvoidingAgent.c2_a * b * (message.data.velocity - obj.velocity);
            end
            
            % Implement the beta agent part of the flocking protocol
            for obst = obj.obstacles
                diff = obj.position - obst.center;
                dist = norm(diff);
                
                % Calculate position and velocity of this beta agent. See
                % Olfati-Saber, 2006, VII. D.
                mu = obst.radius / dist;
                b  = diff / dist;
                P  = eye(2) - b*b';
                qb = mu * obj.position + (1 - mu) * obst.center;
                pb = mu * P * obj.velocity;
                
                % Compute sigma distance between the two agents
                [dist, grad] = sigma_norm(qb - obj.position,...
                                          ObstacleAvoidingAgent.epsilon);
                                      
                % Calculate force on the agent from the virtual potential field
                u = u + ObstacleAvoidingAgent.c1_b...
                        * grad * phi_beta(dist, ObstacleAvoidingAgent.db, ObstacleAvoidingAgent.hb);
                
                % Compute position dependent factor for alignment with beta
                % agent
                b = rho_h(dist / ObstacleAvoidingAgent.db, ObstacleAvoidingAgent.hb);
                
                % Calculate alignment force from beta agent
                u = u + ObstacleAvoidingAgent.c2_b * b * (pb - obj.velocity);
            end
            
            % Define group objective
            [~, sigma] = sigma_norm(obj.q_gamma - obj.position, 1);
            u = u + ObstacleAvoidingAgent.c1_g * sigma + ObstacleAvoidingAgent.c2_g * (obj.p_gamma - obj.velocity);
            
            % Evaluate double integrator dynamics
            obj.move(u);
            
            % Send message to network, include position and velocity.
            % The size of the data is truncated to 16 bits but converted
            % back to double precision to avoid converting all values of
            % the simulation to half precision floating point values.
            data = struct;
            data.position = double(half(obj.position));
            data.velocity = double(half(obj.velocity));
            obj.send(data)
        end
    end
end

