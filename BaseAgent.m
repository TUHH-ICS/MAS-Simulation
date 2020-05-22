classdef(Abstract) BaseAgent < handle
    %BASEAGENT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties(GetAccess = public, SetAccess = protected)
        x
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
            obj.move()
            
            obj.network.setPosition(obj.position);
        end
    end
    
    methods(Abstract, Access = protected)
        move(obj)
    end
end

