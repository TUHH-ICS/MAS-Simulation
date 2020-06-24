classdef SinrNetwork < BaseNetwork
    %SINRNETWORK Class that wraps the C++ networking library from the
    %University of Koblenz for use in this simulation library.
    %   The C++ networking library gets embedded in Matlab as a compiled
    %   MEX function. Different aspects of the library get called using a
    %   string command interface.
    %
    %   It is currently not possible to create multiple instances of the
    %   SinrNetwork class as the MEX function interface has a global
    %   internal state. Multiple instances of the network would corrupt
    %   this state.
    
    properties(GetAccess = public, SetAccess = protected)
        sentMessages % Messages waiting to be processed in the network
    end
    
    properties(GetAccess = public, SetAccess = immutable)
        config(1,1) SinrConfiguration
    end
    
    methods
        function obj = SinrNetwork(config)
            %SINRNETWORK Construct an instance of this class
            %   Initializes the c++ network simulation library and the
            %   Matlab message storage.
            
            % Attempt to aquire the class lock. If the lock was previously
            % set to true, another instance of this class is currently in
            % memory and we can not create another.
            if SinrNetwork.setGetLock(true)
                error('You must only create one instance of the SinrNetwork class at a time!')
            end
            
            obj@BaseNetwork(config.agentCount, config.cycleTime)
            
            % Ensure that the configuration is not changed from the outside
            % after the creation of the network
            obj.config = copy(config);
            
            % Initialize matlab message storage
            obj.sentMessages = MessageBuffer(config.agentCount);
            
            % Build C++ networking library if necessary
            buildSuccess = buildSinrMex();
            if ~buildSuccess
                error('Building SINR networking library failed!')
            end
            
            % Initialize the networking library with the correct number of
            % agents.
            callSinrNetwork('new', config)
        end
        
        function delete(~)
            %DELETE Destructor for class SinrNetwork
            %   Frees the memory that was allocated in the c++ library

            % Call into the C++ library to free its memory
            callSinrNetwork('delete')
            
            % Release SinrNetwork class lock such that another instance can
            % be created.
            SinrNetwork.setGetLock(false);
        end
        
        function updateAgent(obj, agent)
            %UPDATEAGENT Updates the state of the agent, that is contained
            %in the network.
            %   This function updates the state of the agent, as it is seen
            %   by the network. This includes the agent's position and if
            %   it wants to transmit a message.
            
            % Call into the C++ library to update the agent positions
            callSinrNetwork('updateNetworkAgent', agent.id, agent.position)
            
            % Check if the agent wants to transmit
            msg = agent.getMessage();
            if ~isempty(msg)
                obj.sentMessages.put(msg);
            end
        end
        
        function recvMessages = process(obj)
            %PROCESS Processes all messages that were send by the agents
            %since the last call.
            %   The messages get processed in the C++ library using the
            %   SINR model. The library determines only whether a messages
            %   is successfully transmitted, the transmission itself, i. e.
            %   the content of the messages, is handled in Matlab.
            
            % Call into the C++ library to determine all successfully
            % transmitted messages.
            callSinrNetwork('process')
            
            messages = obj.sentMessages.takeAll();
            senders  = [messages.sender];
            recvMessages = cell(obj.agentCount,1);
            
            if ~isempty(messages)
                for i = 1:obj.agentCount
                    % Call into the C++ library to check which messages where
                    % received by agent i in which slot.
                    %
                    % Each row of vec represents a received messages. The first
                    % column contains the id of the sender, the second column
                    % the slot in which the message was received. At the
                    % moment, the slot information is ignored.
                    vec = callSinrNetwork('updateMatlabAgent', i);

                    % Collect all received messages based on the sender
                    mask = any(senders == vec(:, 1));
                    recvMessages{i} = messages(mask);
                end
            end
        end
    end
    
    methods(Static, Access = private)
        function old = setGetLock(lock)
            %SETGETLOCK Function that implements a primitive locking
            %mechanism for the SinrNetwork class.
            %   This function is used to ensure that only one instance of
            %   the SinrNetwork class is created at the same time. The
            %   persistent lockState variable is used to emulate a static
            %   class variable.
            %
            %   To aquire the lock, attempt to set it to true and check
            %   that its previous state was false. To release the lock
            %   unconditionally set it to false.
            
            % Define persistent boolen variable and initialize it to false
            persistent lockState;
            if isempty(lockState)
                lockState = false;
            end
            
            % Return the old state of the lock
            old = lockState;
            
            % If a new state is given as argument, update the lock state
            if nargin >= 1
                lockState = logical(lock);
            end
        end
    end
end

