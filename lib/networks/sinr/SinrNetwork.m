% This file is part of a library for simulation of multi-agent systems
% developed at the Institute of Control Systems at TUHH.
%
% Original Authors: Christian Hespe <christian.hespe@tuhh.de>

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
        recvMessages % Messages waiting to be delivered to the agents
    end
    
    properties(GetAccess = public, SetAccess = immutable)
        config(1,1) SinrConfiguration
        
        % Controls whether the messages that are received in different
        % slots get delivered all at once, or in their respective slot
        enableSubstepping(1,1) logical = false
    end
    
    properties(GetAccess = private, SetAccess = private)
        slotCounter = 1
    end
    
    methods
        function obj = SinrNetwork(config, enableSubstepping)
            %SINRNETWORK Construct an instance of this class
            %   Initializes the c++ network simulation library and the
            %   Matlab message storage.
            
            % Attempt to aquire the class lock. If the lock was previously
            % set to true, another instance of this class is currently in
            % memory and we can not create another.
            if SinrNetwork.setGetLock(true)
                error('You must only create one instance of the SinrNetwork class at a time!')
            end
            
            % Set default value for substepping parameter
            if nargin < 2
                enableSubstepping = false;
            end
            
            % Select correct cycleTime for the network object. When
            % substepping is enabled, the network must be called for each
            % slot, this slotTime is selected.
            if enableSubstepping
                time = config.slotTime;
            else
                time = config.cycleTime;
            end
            
            obj@BaseNetwork(config.agentCount, time)
            
            % Ensure that the configuration is not changed from the outside
            % after the creation of the network
            obj.config = copy(config);
            obj.enableSubstepping = enableSubstepping;
            
            % Initialize matlab message storage
            obj.sentMessages = cell(config.agentCount, 1);
            if enableSubstepping
                obj.recvMessages = cell(config.agentCount, config.slotCount);
            end
            
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
            if ismex('callSinrNetwork')
                callSinrNetwork('delete')
            end
            
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
            callSinrNetwork('updateNetworkAgent', agent.id, double(agent.position))
            
            % Check if the agent wants to transmit
            msg = agent.getMessage();
            if ~isempty(msg)
                % Copy message into the buffer. Any messages that was
                % previously send from this agent since the last cycle will
                % be overriden and thus get dropped.
                obj.sentMessages{agent.id} = msg;
                
                callSinrNetwork('updateSendFlag', agent.id, true);
            end
        end
        
        function recvMessages = process(obj)
            %PROCESS Processes all messages that were send by the agents
            %since the last call.
            %   The messages get processed in the C++ library using the
            %   SINR model. The library determines only whether a messages
            %   is successfully transmitted, the transmission itself, i. e.
            %   the content of the messages, is handled in Matlab.
            
            if obj.slotCounter == 1
                % Call into the C++ library to determine all successfully
                % transmitted messages.
                callSinrNetwork('process')

                % Collect messages that were transmitted since the last
                % call
                messages = [obj.sentMessages{:}];
                obj.sentMessages = cell(obj.agentCount,1);
                
                if ~obj.enableSubstepping || isempty(messages)
                    recvMessages = cell(obj.agentCount, 1);
                end

                if ~isempty(messages)
                    senders = [messages.sender];
                    
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
                        mask = senders == vec(:, 1);
                        if obj.enableSubstepping
                            % Reset received messages from previous cycle
                            obj.recvMessages(i, :) = {[]};
                            
                            % Copy messages into the corresponding slots
                            maskedSlots = vec(any(mask,2), 2);
                            for j = 1:length(maskedSlots)
                                obj.recvMessages{i, maskedSlots(j)} = messages(mask(j,:));
                            end
                        else
                            % Copy all received messages for this agent
                            recvMessages{i} = messages(any(mask,1));
                        end
                    end
                    
                    if obj.enableSubstepping
                        recvMessages = obj.recvMessages(:, 1);
                    end
                else
                    % In the case where no message was sent, the
                    % recvMessages cell contains the wrong data type,
                    % double instead of Message. This line corrects that,
                    % so that consumers of the messages get the correctly
                    % typed object.
                    recvMessages(:) = {Message.empty};
                end 
            else
                % Return all messages that were received in this slot. When
                % substepping is disabled, slotCounter is always 1.
                recvMessages = obj.recvMessages(:, obj.slotCounter);
            end
            
            if obj.enableSubstepping
                obj.slotCounter = mod(obj.slotCounter, obj.config.slotCount) + 1;
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

