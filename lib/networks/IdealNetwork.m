classdef IdealNetwork < MatlabNetwork
    %IDEALNETWORK Network implementation that consideres the connection
    %between agents to be ideal.
    %   This implementation of BaseNetwork does not consider any non-ideal
    %   effects in the network communication but does only transmit
    %   messages over a fixed distance.
    %   Can be used as a baseline and to test simulation under ideal
    %   circumstances.
    
    properties(GetAccess = public, SetAccess = immutable)
        range % Range of the communication
    end
       
    methods
        function obj = IdealNetwork(agentCount, dim, range)
            %IDEALNETWORK Construct an instance of this class
            %   The network needs several parameter to be correctly
            %   initialized.
            %   agentCounter is the number of agents in the network
            %   dim is the dimension of the space the agents move in
            %   range is the communication range
            
            % Initialize MatlabNetwork properties
            obj@MatlabNetwork(agentCount, dim);
            
            obj.range = range;
        end
                
        function process(obj) 
            %PROCESS Processes all messages that were send by the agents
            %since the last call.
            %   The messages get broadcasted to all agents in the receiving
            %   range.
            
            for msg = obj.sendMessages.getAll()
                % Calculate the distance from the sender to all agents
                dist = vecnorm(obj.positions(:, msg.sender) - obj.positions);
                
                % Enumerate all targets inside the receiving range
                idx = find(dist <= obj.range);
                
                % The sender does not receive its own message, so remove it
                % from the list.
                idx(idx == msg.sender) = [];
                
                % Copy the message in all remaining receive buffers
                for id = idx
                    obj.recvMessages{id} = [ obj.recvMessages{id}, msg ];
                end
            end
            
            % All sent messages were process, so clear the queue
            obj.sendMessages.clear();
        end
    end
end