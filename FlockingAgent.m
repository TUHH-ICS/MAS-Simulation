classdef FlockingAgent < BaseAgent
    %FLOCKINGAGENT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties(Constant)
        epsilon = 0.1; % Used to define the sigma_norm
        da      = 7;   % equilibrium distance(or desired distance) to neighbors
        ra      = 1.2 * FlockingAgent.da; % sensing radius (no interaction with agents outside ra disc)
        h       = 0.9; % Bump function goes to zero after h
        c_damp  = 3;   % 
    end
    
    properties(Dependent, GetAccess = public, SetAccess = private)
        position
        velocity
    end
    
    methods     
        function obj = FlockingAgent(network, initialPos, initialVel)
            %FLOCKINGAGENT Construct an instance of this class
            %   Detailed explanation goes here
            
            x0 = kron(initialPos, [1; 0]) + kron(initialVel, [0; 1]);
            obj@BaseAgent(network, x0);
        end
        
        function value = get.position(obj)
            value = [obj.x(1); obj.x(3)];
        end
        
        function value = get.velocity(obj)
            value = [obj.x(2); obj.x(4)];
        end
    end
    
    methods(Access = protected)
        function move(obj)
            T = 0.1;
            
            u = zeros(2, 1);
            for message = obj.messages
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
            obj.messages = [];
            
            obj.x(1) = obj.x(1) + T*obj.x(2);
            obj.x(2) = obj.x(2) + T*u(1);
            obj.x(3) = obj.x(3) + T*obj.x(4);
            obj.x(4) = obj.x(4) + T*u(2);
            
            obj.send()
        end
    end
end

