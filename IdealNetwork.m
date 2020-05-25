classdef IdealNetwork < BaseNetwork
    %IDEALNETWORK Network implementation that consideres the connection
    %between agents to be ideal.
    %   This implementation of BaseNetwork does not consider any non-ideal
    %   effects in the network communication but does only transmit
    %   messages over a fixed distance.
    %   Can be used as a baseline and to test simulation under ideal
    %   circumstances.
    
    properties(GetAccess = public, SetAccess = immutable)
        range        % Range of the communication
    end
    
    properties(GetAccess = public, SetAccess = private)
        positions    % Positions of all agents in the network
        sendMessages % Messages waiting to be processed in the network
        recvMessages % Messages that were passed to the recipients
    end
    
    methods
        function obj = IdealNetwork(agentCount, dim, range)
            %IDEALNETWORK Construct an instance of this class
            %   The network needs several parameter to be correctly
            %   initialized.
            %   agentCounter is the number of agents in the network
            %   dim is the dimension of the space the agents move in
            %   range is the communication range
            
            % Initialize BaseNetwork properties
            obj@BaseNetwork(agentCount);
            
            obj.range        = range;
            obj.positions    = zeros(dim, agentCount);
            
            % Initial message buffers
            % TODO It turns out that the simulation is slower if
            % recvMessage is also switched from cell to MessageBuffer, this
            % should be investigated.
            obj.recvMessages = cell(agentCount,1);
            obj.sendMessages = MessageBuffer(agentCount);
        end
                
        function send(obj, agent, data)
            % SEND Puts the message that is send by the agent into the
            % processing queue.
            
            obj.sendMessages.put(Message(agent.id, data));
        end
        
        function messages = receive(obj, agent)
            % RECEIVE Takes all messages, that were received by the agent
            % and returns them. Each message is only returned once.
            
            messages = obj.recvMessages{agent.id};
            obj.recvMessages{agent.id} = [];
        end
        
        function setPosition(obj, agent)
            % SETPOSITION Updates the internal position of the agents
            
            obj.positions(:, agent.id) = agent.position;
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