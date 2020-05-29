classdef(Abstract) MatlabNetwork < BaseNetwork
    %MATLABNETWORK Common base class for networks that are completely
    %implemented in Matlab.
    %   This class bundles the common tasks of a Matlab network
    %   implementation, such as storing sent and received messages and
    %   managing the positions of the agents.
    %   Only the processing of the messages is implementation specific.

    properties(GetAccess = public, SetAccess = protected)
        positions    % Positions of all agents in the network
        sendMessages % Messages waiting to be processed in the network
        recvMessages % Messages that were passed to the recipients
    end
    
    methods
        function obj = MatlabNetwork(agentCount, dim)
            %MATLABNETWORK Construct an instance of this class
            %   The network needs several parameter to be correctly
            %   initialized.
            %   agentCounter is the number of agents in the network
            %   dim is the dimension of the space the agents move in
            
            % Initialize BaseNetwork properties
            obj@BaseNetwork(agentCount);
            
            % Allocate space to save agent positions
            obj.positions    = zeros(dim, agentCount);
            
            % Initial message buffers
            % TODO It turns out that the simulation is slower if
            % recvMessage is also switched from cell to MessageBuffer, this
            % should be investigated.
            obj.recvMessages = cell(agentCount,1);
            obj.sendMessages = MessageBuffer(agentCount);
        end
                
        function send(obj, agent, data)
            % SEND Puts the message that is send by the agent into the
            % processing queue.
            
            obj.sendMessages.put(Message(agent.id, data));
        end
        
        function messages = receive(obj, agent)
            % RECEIVE Takes all messages, that were received by the agent
            % and returns them. Each message is only returned once.
            
            messages = obj.recvMessages{agent.id};
            obj.recvMessages{agent.id} = [];
        end
        
        function setPosition(obj, agent)
            % SETPOSITION Updates the internal position of the agents
            
            obj.positions(:, agent.id) = agent.position;
        end
    end
end