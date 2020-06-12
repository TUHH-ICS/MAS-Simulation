classdef SinrNetwork < BaseNetwork
    %SINRNETWORK Summary of this class goes here
    %   Detailed explanation goes here
    
    properties(GetAccess = public, SetAccess = protected)
        sendMessages % Messages waiting to be processed in the network
        recvMessages % Messages that were passed to the recipients
    end
    
    properties(GetAccess = public, SetAccess = immutable)
        dimension
    end
    
    methods
        function obj = SinrNetwork(agentCount, dim)
            %SINRNETWORK Construct an instance of this class
            %   Initializes the c++ network simulation library
            
            obj@BaseNetwork(agentCount)
            obj.dimension = dim;
            
            obj.recvMessages = cell(agentCount,1);
            obj.sendMessages = MessageBuffer(agentCount);
            
            matlab_adapter('new', agentCount)
        end
        
        function delete(~)
            %DELETE Destructor for class SinrNetwork
            %   Frees the memory that was allocated in the c++ library

            matlab_adapter('delete')
        end
        
        function send(obj, agent, data)
            obj.sendMessages.put(Message(agent.id, data));
        end
        
        function messages = receive(obj, agent)
            messages = obj.recvMessages{agent.id};
            obj.recvMessages{agent.id} = [];
        end
        
        function setPosition(obj, agent)
            posVec = agent.position;
            if obj.dimension < 3
                posVec = [ posVec; zeros(3-obj.dimension) ];
            end
            
            matlab_adapter('updateNetworkAgent', agent.id, posVec)
        end
        
        function process(obj)
            matlab_adapter('process')
            
            messages = obj.sendMessages.getAll();
            
            for i = 1:obj.agentCount
                vec = matlab_adapter('updateMatlabAgent', i);
                obj.recvMessages{i} = [ obj.recvMessages{i}, messages(vec(:, 1)) ];
            end
            
            obj.sendMessages.clear();
        end
    end
end

