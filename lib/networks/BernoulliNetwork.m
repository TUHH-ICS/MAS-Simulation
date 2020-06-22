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
        function obj = BernoulliNetwork(agentCount, cycleTime, dim, range, p)
            %BERNOULLINETWORK Construct an instance of this class
            %   The network needs several parameter to be correctly
            %   initialized.
            %   agentCounter is the number of agents in the network
            %   dim is the dimension of the space the agents move in
            %   range is the communication range
            %   p is the probability of a successful transmission
            
            % Initialize MatlabNetwork properties
            obj@MatlabNetwork(agentCount, cycleTime, dim);
            
            obj.range = range;
            obj.p     = p;
        end
        
        function recvMessages = process(obj) 
            %PROCESS Processes all messages that were send by the agents
            %since the last call.
            %   The messages get broadcasted to all agents in the receiving
            %   range. Some percentage of the messages gets dropped
            %   randomly.
            
            sentMessages = obj.sentMessages.takeAll();
            
            % Exclude the possibility of agents sending to themselves
            filter = ~eye(length(sentMessages), obj.agentCount, 'logical');
            
            % Compute the recipients of each message
            for i = 1:length(sentMessages)
                pos_sender = obj.positions(:, sentMessages(i).sender);
                
                % Calculate the distance from the sender to all agents
                dist = vecnorm(pos_sender - obj.positions);
                
                % Remove receivers that are outside the transmission range
                filter(i,:) = filter(i,:) & (dist <= obj.range);
            end
            
            % Drop messages randomly -> Bernoulli distribution
            filter = filter & (rand(size(filter)) <= obj.p);
            
            % Copy all received messages in the receiving buffers
            recvMessages = cell(obj.agentCount,1);
            for i = 1:obj.agentCount
                recvMessages{i} = sentMessages(filter(:,i));
            end
        end
    end
end