%---------------------------------------------------------------------------------------------------
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Christian Hespe
%---------------------------------------------------------------------------------------------------

classdef(Abstract) BaseAgent < handle & matlab.mixin.Heterogeneous
    %BASEAGENT Defines the basic properties of a agent in the simulation
    %framework.
    %   All specific agent classes must inherit from this abstract class,
    %   as it defines the basis properties, that are required during
    %   simulation.
    
    properties(GetAccess = public, SetAccess = immutable)
        id         % Number that uniquely identifies the agent in the network
        dT         % Time between to calls to the step function
    end
    
    properties(GetAccess = private, SetAccess = private)
        dataToSend % Saves the data the agent wants to transmitt until the networks fetches it
        messages   % Buffer for received messages of this agent
    end

    properties(Dependent, GetAccess = public, SetAccess = immutable)
        order      % Dynamic order, dimension of the state space of the agent
    end
    
    properties(Abstract, Dependent, GetAccess = public, SetAccess = private)
        % These dependent properties are only there for convenience. Because
        % the position and velocity can be encoded in arbitrary states, this is
        % used to easy working with the agents without increasing the storage
        % cost.
        position % Current position of the agent
        velocity % Current velocity of the agent
        
        % The state is also saved as a dependent property. In this way, the
        % agent implementation can choose how to implement the dynamics.
        % Either, the predefined dynamics objects can be used, which
        % contain a state themselves, or it can choose to declare a local
        % variable and manually implement the dynamics
        state    % Dynamic state of the agent
    end
    
    methods
        function obj = BaseAgent(id, dT)
            %BASEAGENT Construct an instance of this class
            %   network is a reference to the network object.
            %   dT is the time between two simulation steps

            obj.id = id;
            obj.dT = dT;
        end
        
        function value = get.order(obj)
            %GET.ORDER Implementation of the dependent property order
            
            value = size(obj.state, 1);
        end
        
        function msgs = receive(obj, ~, msgs)
            %RECEIVE Function that agent implementation can call to receive
            %messages from other agents in the network.
            %   This default implementation is meant to be used if the
            %   agent only processes messages in its own step function. If
            %   processing between steps is required, you must override
            %   this function.
            
            if isempty(obj.messages)
                obj.messages = MessageBuffer();
            end
            
            if nargin > 1 && nargout == 0                
                obj.messages.put(msgs);
            elseif nargin <= 1 && nargout > 0
                msgs = obj.messages.takeAll();
            else
                error('You must have either inputs or an output!')
            end            
        end
    end
    
    methods(Abstract)
        %STEP Function that gets called at each simulation step
        %   All agent implementations should override this method to
        %   implement agent specific behaviour, such as the dynamics
        %   and the network interactions.
        step(obj)
    end

    methods(Access = protected)
        function send(obj, data)
            %SEND Function that the agent implementations can call to send
            %data over the network
            %   The data is not send immediately, but stored until the
            %   network requests it.
            
            obj.dataToSend = data;
        end
    end
    
    methods(Access = ?BaseNetwork)
        function message = getMessage(obj)
            %GETMESSAGE Function that returns the message that this agent
            %wants to send.
            %   If no data is to be sent, the reponse is empty.
            
            if isempty(obj.dataToSend)
                message = [];
            else
                message = Message(obj.id, obj.dataToSend);
                obj.dataToSend = [];
            end
        end
    end
    
    methods (Static, Sealed, Access = protected)
        function default_object = getDefaultScalarElement()
            %GETDEFAULTSCALARELEMENT Provides a default element for Matlab
            %to place in newly created arrays of BaseAgents.
            
            default_object = StationaryAgent;
        end
    end
end
