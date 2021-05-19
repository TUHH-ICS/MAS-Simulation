% This file is part of a library for simulation of multi-agent systems
% developed at the Institute of Control Systems at TUHH.
%
% Original Authors: Christian Hespe <christian.hespe@tuhh.de>

classdef MarkovNetwork < MatlabNetwork
    %MARKOVNETWORK Network implementation that models paket loss using a
    %two-state Markov chain.
    %   The network implements a 2-state Markov chain for each channel. One
    %   state for a failed channel, one for a good channel. At each
    %   sampling instance, the chain is advanced in time according to the
    %   given transition probabilities p_fail and p_recover.
    
    properties(GetAccess = public, SetAccess = immutable)
        range     % Range of the communication
        p_fail    % Probability that a good channels fails
        p_recover % Probability that a failed channel recovers
        symmetric % If true, only symmetric packet loss is considered, i.e.
                  % if one direction fails, the other does as well.
    end
    
    properties(GetAccess = public, SetAccess = private)
        state     % State of the Markov chain. This is a boolean variable,
                  % not a probability distribution, as we are considering
                  % only a single realization of the Markov chain here.
    end
    
    methods
        function obj = MarkovNetwork(agentCount, cycleTime, dim, range, p_fail, p_recover, init, symmetric)
            %MARKOVNETWORK Construct an instance of this class
            %   The network needs several parameter to be correctly
            %   initialized.
            %
            %   agentCount    Number of agents in the network
            %   dim           Dimension of the underlying space
            %   range         Communication range
            %   p_fail        Probability that a good channels fails
            %   p_recover     Probability that a failed channel recovers
            %   init          Initial state of the channels, default: []
            %                 Give either a probability for a good initial
            %                 transmission or a matrix with the state
            %   symmetric     Symmetric packet loss, default: false
            
            % Initialize MatlabNetwork properties
            obj@MatlabNetwork(agentCount, cycleTime, dim);
            
            obj.range     = range;
            obj.p_fail    = p_fail;
            obj.p_recover = p_recover;
                        
            % Default to asymmetric channels
            if nargin >= 8
                obj.symmetric = symmetric;
            else
                obj.symmetric = false;
            end
            
            % Initialize the state of the channels
            if nargin >= 7 && ~isempty(init) && length(init) > 1
                % Check size compatability
                [n,m] = size(init);
                if n ~= m || n ~= agentCount
                    error('init must be a square matrix of size agentCount')
                end
                
                % Check symmetry
                if obj.symmetric && any(xor(init, init'), 'all')
                    error('init must be symmetric')
                end
                
                obj.state = logical(init);
            else
                probs = rand(agentCount);
                
                % Ensure symmetric initialization if requested
                if obj.symmetric
                    probs = triu(probs) + triu(probs, 1)';
                end
                
                % Use failure probability for initial state if no
                % probability is given
                if nargin < 7 || isempty(init)
                    obj.state = probs > p_fail;
                else
                    obj.state = probs < init;
                end
            end
        end
        
        function recvMessages = process(obj) 
            %PROCESS Processes all messages that were send by the agents
            %since the last call.
            %   The messages get broadcasted to all agents in the receiving
            %   range. Some messages are dropped according to the
            %   development of a 2-state Markov chain.
            
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
            
            % Draw random matrix for channel state update
            probs = rand(obj.agentCount);
            if obj.symmetric
                % Make random matrix symmetric for symmetric packet loss
                probs = triu(probs) + triu(probs, 1)';
            end
            
            % If an entry of state is TRUE, the channel was good in the
            % last step. Therefore, we set it to 0 with probability p_fail.
            % If it is 0 on the other hand, we set it to 1 with probability
            % p_recover.
            obj.state = ( obj.state & (obj.p_fail    < probs)) ...
                      | (~obj.state & (obj.p_recover > probs));
            
            % Drop messages according to the new channel states. Relevant
            % are only the rows corresponding to the senders of the
            % messages.
            filter = filter & obj.state([sentMessages.sender], :);
            
            % Copy all received messages in the receiving buffers
            recvMessages = cell(obj.agentCount,1);
            for i = 1:obj.agentCount
                recvMessages{i} = sentMessages(filter(:,i));
            end
        end
    end
end