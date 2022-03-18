%---------------------------------------------------------------------------------------------------
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Christian Hespe
%---------------------------------------------------------------------------------------------------

classdef ConsensusAgent < IntegratorAgent
    %CONSENSUSAGENT Simple agent that implements a first order consensus
    %protocol.
    
    properties(Constant)
        epsilon = 2; % Convergence speed of the consensus protocol
    end
    
    properties(GetAccess = public, SetAccess = immutable)
        neighbours;  % Agents this agent should receive information from
    end
    
    methods
        function obj = ConsensusAgent(id, dT, initialValue, neighbours)
            %CONSENSUSAGENT Construct an instance of this class
            %   Sets up the correct agent dynamics and initializes the
            %   agent and consensus protocol to the given initial position.
            %
            %   The optional neighbours argument allows to restrict which
            %   other agents this one is listening to. It should contain a
            %   list of agent ids or be empty.
            
            obj@IntegratorAgent(id, dT, initialValue);
            
            if nargin <= 3
                obj.neighbours = [];
            else
                % (:) converts a vector into a column vector, no matter
                % which format it had before. Important for the later
                % comparison with sender ids.
                obj.neighbours = neighbours(:);
            end
        end
        
        function step(obj)
            % Receive messages from the network
            messages = obj.receive();
            
            % Filter out messages from agents that are no neighbours
            if ~isempty(messages) && ~isempty(obj.neighbours)
                messages(~any([messages.sender] == obj.neighbours)) = [];
            end
            
            d = zeros(obj.dim, 1);
            if ~isempty(messages)
                % Calculate the disagreement vector
                data   = [messages.data];
                values = [data.value];
                d      = sum(values - obj.position, 2);
            end
            
            % Evaluate agent dynamics
            obj.move(obj.epsilon * d);
            
            % Send message to network, include only the position 
            data = struct;
            data.value = obj.position;
            obj.send(data)
        end
    end
end
