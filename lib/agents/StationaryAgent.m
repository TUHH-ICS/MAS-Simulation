classdef StationaryAgent < BaseAgent
    %STATIONARYAGENT A default agent implementation.
    %   This agent has no intended purpose other being a reasonable filling
    %   element for Matlab, when it wants to create BaseAgent arrays. It
    %   does not move ever, and simple drops all received messages without
    %   handling them.
    
    properties(GetAccess = private, SetAccess = immutable)
        x        % Local state, only contains the constant position
    end
    
    properties(Dependent, GetAccess = public, SetAccess = private)
        position % Current position of the agent
        velocity % Current velocity of the agent
        state    % Dynamic state of the agent
    end
    
    methods     
        function obj = StationaryAgent(network, dT, pos)
            %STATIONARYAGENT Construct an instance of this class
            
            % In case the constructor is called without arguments, provide
            % default values. For example, this happens when Matlab tries
            % to construct a default element for a BaseAgent array.
            if nargin == 0
                network = [];
            end
            if nargin <= 1
                dT = -1;
            end
            if nargin <= 2
                pos = 0;
            end
            
            obj@BaseAgent(network, dT);
            obj.x = pos;
        end
        
        function value = get.state(obj)
            %GET.STATE Implementation of the dependent state property.
            value = obj.x;
        end
        
        function value = get.position(obj)
            %GET.POSITION Implementation of the dependent position
            %property.
            %   The state of this agent does only consist of its position.
            value = obj.x;
        end
        
        function value = get.velocity(obj)
            %GET.VELOCITY Implementation of the dependent velocity
            %property.
            %   As this agent is stationary, the velocity is always 0.
            value = zeros(size(obj.x));
        end
        
        function step(obj)
            % Drop received messages
            obj.network.receive();
            
            % Execute BaseAgent step
            obj.step@BaseAgent()
        end
    end
end

