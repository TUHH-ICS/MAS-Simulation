classdef(Abstract) BaseNetwork < handle
    %BASENETWORK Defines the basis properties of a communication network in
    %this simulation framework.
    %   All specific network implementations need to inherit from this
    %   class to be compatible with the agent classes.
    
    properties(Access = protected)
        idCounter % Contains the next id that will be handed out
    end
    
    properties(GetAccess = public, SetAccess = immutable)
        agentCount % Number of agents that the network supports
    end
    
    methods
        function obj = BaseNetwork(agentCount)
            %BASENETWORK Construct an instance of this class.
            %   The number of agents that this network instance can support
            %   must be set here and cannot be changed later.
            
            obj.agentCount = agentCount;
            
            % The ids are given out sequentially, starting with 1.
            obj.idCounter = 1;
        end
        
        function id = getId(obj)
            %GET_ID Request a unique id from the network
            %   This function hands out unique ids for the agents. These
            %   ids are required for sending and receiving messages. If
            %   more ids are request than agents are configured, an error
            %   is returned and the simulation terminates.
            
            if obj.idCounter > obj.agentCount
                error('You registered more agents than you initially requested')
            end
            
            % Return next sequential id
            id = obj.idCounter;
            obj.idCounter = obj.idCounter + 1;
        end
    end
    
    methods(Abstract)
        % Method that can be called by the agent implementations to send a
        % message over the network. All messages are broadcasted in this
        % framework, so there is no receiver
        send(obj, agent, data)
        
        % Method that can be called by the agent implementations to receive
        % messages from other agents. Every received message should only be
        % returned once and is then dropped from the internal storage of
        % the network.
        messages = receive(obj, agent)
        
        % Method that must be called by each agent after it changed its
        % position. This position data is required to determine the
        % recipients of the messages.
        setPosition(obj, agent)
        
        % Method that gets called by the simulation framework. It should be
        % used for processing all send messages and putting them in the
        % corresponding receiving slot.
        process(obj)
    end
end

