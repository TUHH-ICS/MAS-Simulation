%---------------------------------------------------------------------------------------------------
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Christian Hespe
%---------------------------------------------------------------------------------------------------

classdef DelayNetwork < MatlabNetwork
    %DELAYNETWORK Network implementation that simulates a constant and uniform delay during
    %transmission
    %   This implementation of BaseNetwork does not consider any stochastic
    %   effects during transmission, but is subject to a constant and
    %   uniform delay for all informaton transmitted through the net
    
    properties(GetAccess = public, SetAccess = immutable)
        range % Range of the communication
        delay % Transmission delay
    end

    properties(GetAccess = private, SetAccess = private)
        buffer % Delay buffer
        idx    % buffer index
    end

    methods
        function obj = DelayNetwork(agentCount, cycleTime, dim, range, delay)
            %DELAYNETWORK Construct an instance of this class
            %   The network needs several parameter to be correctly
            %   initialized.
            %
            %   agentCount    Number of agents in the network
            %   dim           Dimension of the underlying space
            %   range         Communication range
            %   delay         Communication delay in steps
            
            % Initialize MatlabNetwork properties
            obj@MatlabNetwork(agentCount, cycleTime, dim);
            
            obj.range = range;
            obj.delay = delay;

            obj.idx    = 1;
            obj.buffer = cell(agentCount, delay+1);
        end
                
        function recvMessages = process(obj) 
            %PROCESS Processes all messages that were send by the agents
            %since the last call.
            %   The messages get broadcasted to all agents in the receiving
            %   range.
            
            sentMessages = obj.sentMessages.takeAll();
            
            % Initialize every message as received by everyone
            filter = ones([length(sentMessages), obj.agentCount], 'logical');
            
            % Compute the recipients of each message
            for i = 1:length(sentMessages)
                pos_sender = obj.positions(:, sentMessages(i).sender);
                
                % Exclude the possibility of agents sending to themselves
                filter(i, sentMessages(i).sender) = false;
                
                % Calculate the distance from the sender to all agents
                dist = vecnorm(pos_sender - obj.positions);
                
                % Remove receivers that are outside the transmission range
                filter(i,:) = filter(i,:) & (dist <= obj.range);
            end
            
            % Copy all received messages in the delay buffers
            for i = 1:obj.agentCount
                obj.buffer{i,obj.idx} = sentMessages(filter(:,i));
            end

            % Prepare receive buffer and advance the index
            recvMessages = obj.buffer(:, obj.mask(obj.idx-1));
            obj.idx      = obj.mask(obj.idx+1);
        end
    end

    methods(Access = private)
        function idx = mask(obj, idx)
            idx = mod(idx-1, obj.delay+1) + 1;
        end
    end
end
