classdef IdealNetwork < BaseNetwork
    %IDEALNETWORK Summary of this class goes here
    %   Detailed explanation goes here
    
    properties(GetAccess = public, SetAccess = immutable)
        range
    end
    
    properties(GetAccess = public, SetAccess = private)
        positions    % Positions of all agents in the network
        sendMessages % Messages waiting to be processed in the network
        recvMessages % Messages that were passed to the recipients
    end
    
    methods
        function obj = IdealNetwork(agentCount, dim, range)
            %IDEALNETWORK Construct an instance of this class
            %   Detailed explanation goes here
            
            obj@BaseNetwork(agentCount);
            
            obj.range        = range;
            obj.positions    = zeros(dim, agentCount);
            obj.recvMessages = cell(agentCount, 1);
            obj.sendMessages = [];
        end
                
        function send(obj, agent, data)
            obj.sendMessages = [ obj.sendMessages, Message(agent.id, data) ];
        end
        
        function messages = receive(obj, agent)
            messages = obj.recvMessages{agent.id};
            obj.recvMessages{agent.id} = [];
        end
        
        function setPosition(obj, agent)
            obj.positions(:, agent.id) = agent.position;
        end
        
        function process(obj)          
            for msg = obj.sendMessages
                % Calculate the distance from the sender to all agents
                dist = vecnorm(obj.positions(:, msg.sender) - obj.positions);
                
                % Enumerate all targets inside the receiving range
                idx = find(dist <= obj.range);
                
                % The sender does not receive its own message, so remove it
                % from the list.
                idx(idx == msg.sender) = [];
                
                for id = idx
                    obj.recvMessages{id} = [ obj.recvMessages{id}, msg ];
                end
            end
            
            obj.sendMessages = [];
        end
    end
end