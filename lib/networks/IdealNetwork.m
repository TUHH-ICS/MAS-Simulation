%---------------------------------------------------------------------------------------------------
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Christian Hespe
%---------------------------------------------------------------------------------------------------

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
        function obj = IdealNetwork(agentCount, cycleTime, dim, range)
            %IDEALNETWORK Construct an instance of this class
            %   The network needs several parameter to be correctly
            %   initialized.
            %
            %   agentCount    Number of agents in the network
            %   dim           Dimension of the underlying space
            %   range         Communication range
            
            % Initialize MatlabNetwork properties
            obj@MatlabNetwork(agentCount, cycleTime, dim);
            
            obj.range = range;
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
            
            % Copy all received messages in the receiving buffers
            recvMessages = cell(obj.agentCount,1);
            for i = 1:obj.agentCount
                recvMessages{i} = sentMessages(filter(:,i));
            end
        end
    end
end
