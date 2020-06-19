classdef(Abstract) MatlabNetwork < BaseNetwork
    %MATLABNETWORK Common base class for networks that are completely
    %implemented in Matlab.
    %   This class bundles the common tasks of a Matlab network
    %   implementation, such as storing sent and received messages and
    %   managing the positions of the agents.
    %   Only the processing of the messages is implementation specific.

    properties(GetAccess = public, SetAccess = protected)
        positions    % Positions of all agents in the network
        sentMessages % Messages waiting to be processed in the network
    end
    
    methods
        function obj = MatlabNetwork(agentCount, cycleTime, dim)
            %MATLABNETWORK Construct an instance of this class
            %   The network needs several parameter to be correctly
            %   initialized.
            %   agentCounter is the number of agents in the network
            %   dim is the dimension of the space the agents move in
            
            % Initialize BaseNetwork properties
            obj@BaseNetwork(agentCount, cycleTime);
            
            % Allocate space to save agent positions
            obj.positions    = zeros(dim, agentCount);
            
            % Initial message buffers
            obj.sentMessages = MessageBuffer(agentCount);
        end
        
        function updateAgent(obj, agent)
            %UPDATEAGENT Updates the state of the agent, that is contained
            %in the network.
            %   This function updates the state of the agent, as it is seen
            %   by the network. This includes the agent's position and if
            %   it wants to transmit a message.
            
            % Update the position of the agent
            obj.positions(:, agent.id) = agent.position;
            
            % Check if the agent wants to transmit
            msg = agent.getMessage();
            if ~isempty(msg)
                obj.sentMessages.put(msg);
            end
        end
    end
end