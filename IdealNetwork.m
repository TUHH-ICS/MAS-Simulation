classdef IdealNetwork < BaseNetwork
    %IDEALNETWORK Summary of this class goes here
    %   Detailed explanation goes here
        
    properties(GetAccess = public, SetAccess = immutable)
        Range
    end
    
    properties(GetAccess = public, SetAccess = private)
        AgentPositions
        SendMessages
        RecvMessages
    end
    
    properties(Dependent, GetAccess = public, SetAccess = private)
        NumAgents
    end
    
    methods
        function obj = IdealNetwork(agentCount, dim, range)
            %IDEALNETWORK Construct an instance of this class
            %   Detailed explanation goes here

            obj@BaseNetwork(agentCount);
            
            obj.Range          = range;
            obj.AgentPositions = zeros(dim, agentCount);
            obj.RecvMessages   = cell(agentCount, 1);
            obj.SendMessages   = [];
        end
        
        function value = get.NumAgents(obj)
            value = size(obj.Agents, 2);
        end
                
        function send(obj, agent, data)
            obj.SendMessages = [ obj.SendMessages, Message(agent.Id, data) ];
        end
        
        function messages = receive(obj, agent)
            messages = obj.RecvMessages{agent.Id};
            obj.RecvMessages{agent.Id} = [];
        end
        
        function setPosition(obj, agent)
            obj.AgentPositions(:, agent.Id) = agent.position;
        end
        
        function process(obj)
            filter = zeros(obj.NumAgents);
            
            for msg = obj.SendMessages
                % Enumerate all possible targets
                idx = 1:obj.NumAgents;
                
                % Remove sender from list
                idx(idx == msg.Sender) = [];
                
                for id = idx
                    received = false;
                    
                    % We make the assumption that the connection is
                    % symmetric. Therefore we only need to test half the
                    % time.
                    [a, b] = asc(msg.Sender, id);
                    
                    if filter(a,b) == 1 % Agent in range
                        received = true;
                    elseif filter(a,b) == 0 % Agent not yet tested
                        sendPos = obj.AgentPositions(:, msg.Sender);
                        recvPos = obj.AgentPositions(:, id);
                        
                        if norm(sendPos - recvPos) <= range
                            filter(a,b) = 1;
                            received    = true;
                        else
                            filter(a,b) = -1;
                        end
                    end
                    
                    % Message successfully received
                    if received
                        obj.RecvMessage{id} = [ obj.RecvMessage{id}, msg ];
                    end
                end
            end
            
            obj.SendMessages = [];
        end
    end
end

function [a, b] = asc(x, y)
%ASC Sorts the two arguments in ascending order
    if x > y
        a = y;
        b = x;
    else
        a = x;
        b = y;
    end
end
