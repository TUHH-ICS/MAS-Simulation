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
            
            messages = obj.sendMessages.getAll();
            
            % Exclude the possibility of agents sending to themselves
            filter = ~eye(length(messages), obj.agentCount, 'logical');
            
            % Compute the recipients of each message
            for i = 1:length(messages)
                pos_sender = obj.positions(:, messages(i).sender);
                
                % Calculate the distance from the sender to all agents
                dist = vecnorm(pos_sender - obj.positions);
                
                % Remove receivers that are outside the transmission range
                filter(i,:) = filter(i,:) & (dist <= obj.range);
            end
            
            % Copy all received messages in the receiving buffers
            for i = 1:obj.agentCount
                obj.recvMessages{i} = [ obj.recvMessages{i}, messages(filter(:,i)) ];
            end
            
            % All sent messages were processed, so clear the queue
            obj.sendMessages.clear();
        end
    end
end