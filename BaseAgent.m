classdef(Abstract) BaseAgent < handle
    %BASEAGENT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties(GetAccess = public, SetAccess = protected)
        x
        messages
    end
        
    properties(GetAccess = public, SetAccess = immutable)
        id
    end
    
    properties(GetAccess = private, SetAccess = immutable)
        network
    end
    
    properties(Dependent, GetAccess = public, SetAccess = immutable)
        nx
    end
    
    properties(Abstract, Dependent, GetAccess = public, SetAccess = private)
        position
        velocity
    end
    
    methods
        function obj = BaseAgent(network, x0)
            %BASEAGENT Construct an instance of this class
            %   Detailed explanation goes here
            
            obj.network = network;
            obj.id      = network.getId();
            obj.x       = x0;
        end
        
        function value = get.nx(obj)
            value = size(obj.x, 1);
        end
    end
    
    methods(Sealed)
        function step(obj)
            obj.messages = [ obj.messages, obj.network.receive(obj) ];
            obj.move()
            obj.network.setPosition(obj);
        end
    end
    
    methods(Sealed, Access = protected)
        function send(obj)
            data = struct;
            data.position = obj.position;
            data.velocity = obj.velocity;
            obj.network.send(obj, data)
        end
    end
    
    methods(Abstract, Access = protected)
        move(obj)
    end
end

