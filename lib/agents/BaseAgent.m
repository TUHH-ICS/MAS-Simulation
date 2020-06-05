classdef(Abstract) BaseAgent < handle & matlab.mixin.Heterogeneous
    %BASEAGENT Defines the basic properties of a agent in the simulation
    %framework.
    %   All specific agent classes must inherit from this abstract class,
    %   as it defines the basis properties, that are required during
    %   simulation.
    
    properties(GetAccess = public, SetAccess = private)
        t        % Current simulation time
    end
    
    properties(GetAccess = public, SetAccess = immutable)
        id       % Number that uniquely identifies the agent in the network
        dT       % Time between to calls to the step function
    end
    
    properties(GetAccess = private, SetAccess = immutable)
        network  % Pointer to the network object, for sending and receiving
    end
    
    properties(Dependent, GetAccess = public, SetAccess = immutable)
        order    % Dynamic order, dimension of the state space of the agent
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
        function obj = BaseAgent(network, dT)
            %BASEAGENT Construct an instance of this class
            %   network is a reference to the network object.
            %   dT is the time between two simulation steps
            
            obj.network = network;
            obj.t       = 0;
            obj.dT      = dT;
            
            % Test if a correct network implementation was handed in
            if isa(network, 'BaseNetwork')
                obj.id  = network.getId(); % Aquire unique id
            end
        end
        
        function value = get.order(obj)
            %GET.ORDER Implementation of the dependent property order
            value = size(obj.state, 1);
        end
        
        function step(obj)
            %STEP Function that gets called at each simulation step
            %   All agent implementations should override this method to
            %   implement agent specific behaviour, such as the dynamics
            %   and the network interactions.
            %   
            %   You should call the superclass method at the end of your
            %   custom implementation
            
            % Update agent position in the network. This is independent of
            % sending a message that may contain the agent position.
            obj.network.setPosition(obj);
            
            % Update current simulation time
            obj.t = obj.t + obj.dT;
        end
    end
    
    methods(Sealed, Access = protected)
        function send(obj, data)
            %SEND Function that the agent implementations can call to send
            %data over the network
            obj.network.send(obj, data)
        end
        
        function messages = receive(obj)
            %RECEIVE Function that agent implementation can call to receive
            %messages from other agents in the network
            messages = obj.network.receive(obj);
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

