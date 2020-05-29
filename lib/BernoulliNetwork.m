classdef BernoulliNetwork < MatlabNetwork
    %BERNOULLINETWORK Network implementation that models paket loss using a
    %simple bernoulli distribution.
    %   The network has a fixed and uniform transmission probability p,
    %   that controls with which probability a message is receive if the
    %   recipient is in the transmission range.
    
    properties(GetAccess = public, SetAccess = immutable)
        range % Range of the communication
        p     % Probability of a successful transmission
    end
    
    methods
        function obj = BernoulliNetwork(agentCount, dim, range, p)
            %BERNOULLINETWORK Construct an instance of this class
            %   The network needs several parameter to be correctly
            %   initialized.
            %   agentCounter is the number of agents in the network
            %   dim is the dimension of the space the agents move in
            %   range is the communication range
            %   p is the probability of a successful transmission
            
            % Initialize MatlabNetwork properties
            obj@MatlabNetwork(agentCount, dim);
            
            obj.range = range;
            obj.p     = p;
        end
        
        function process(obj) 
            %PROCESS Processes all messages that were send by the agents
            %since the last call.
            %   The messages get broadcasted to all agents in the receiving
            %   range.
            
            for msg = obj.sendMessages.getAll()
                % Calculate the distance from the sender to all agents
                dist = vecnorm(obj.positions(:, msg.sender) - obj.positions);
                
                % Generate random numbers to determine if the message was
                % transmitted successfully to each possible recepient
                pTransmit = rand(size(dist));
                
                % Enumerate all targets that are inside the receiving range
                % and additionally the transmission succeeded
                idx = find((dist <= obj.range) & (pTransmit <= obj.p));
                
                % The sender does not receive its own message, so remove it
                % from the list
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