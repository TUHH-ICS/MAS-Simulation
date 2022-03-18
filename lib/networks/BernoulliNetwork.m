%---------------------------------------------------------------------------------------------------
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Christian Hespe
%---------------------------------------------------------------------------------------------------

classdef BernoulliNetwork < MatlabNetwork
    %BERNOULLINETWORK Network implementation that models paket loss using a
    %simple Bernoulli distribution.
    %   The network has a fixed and uniform transmission probability p,
    %   that controls with which probability a message is receive if the
    %   recipient is in the transmission range.
    
    properties(GetAccess = public, SetAccess = immutable)
        range     % Range of the communication
        p         % Probability of a successful transmission
        symmetric % If true, only symmetric packet loss is considered, i.e.
                  % if one direction fails, the other does as well.
    end
    
    methods
        function obj = BernoulliNetwork(agentCount, cycleTime, dim, range, p, symmetric)
            %BERNOULLINETWORK Construct an instance of this class
            %   The network needs several parameter to be correctly
            %   initialized.
            %
            %   agentCount    Number of agents in the network
            %   dim           Dimension of the underlying space
            %   range         Communication range
            %   p             Probability of a successful transmission
            %                 Can either be a scalar for uniform loss
            %                 probability across links, or a NxN matrix
            %                 with N being the agentCount
            %   symmetric     Symmetric packet loss, default: false
            
            % Initialize MatlabNetwork properties
            obj@MatlabNetwork(agentCount, cycleTime, dim);
            
            obj.range = range;
            obj.p     = p;
                        
            % Default to asymmetric channels
            if nargin >= 6
                obj.symmetric = symmetric;
            else
                obj.symmetric = false;
            end
            
            % Check compatability of p
            if ~isscalar(p)
                if dim(p) == 2 && all(size(p) == agentCount)
                    if obj.symmetric && ~issymmetric(p)
                        error('p needs to be symmetric for symmetric loss')
                    end
                else
                    error('p must either be scalar or NxN')
                end
            end
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
            probs = rand(obj.agentCount);
            if obj.symmetric
                % Make random matrix symmetric for symmetric packet loss
                probs = triu(probs) + triu(probs, 1)';
            end
            
            % Extract rows that correspond to the sender of each message
            probs = probs([sentMessages.sender], :);
            filter = filter & (probs < obj.p);
            
            % Copy all received messages in the receiving buffers
            recvMessages = cell(obj.agentCount,1);
            for i = 1:obj.agentCount
                recvMessages{i} = sentMessages(filter(:,i));
            end
        end
    end
end
