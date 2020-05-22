classdef FlockingAgent < BaseAgent
    %FLOCKINGAGENT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties(GetAccess = public, SetAccess = private)
        u
    end
    
    properties(Dependent, GetAccess = public, SetAccess = private)
        position
        velocity
    end
    
    methods     
        function obj = FlockingAgent(network, initialPosition)
            %FLOCKINGAGENT Construct an instance of this class
            %   Detailed explanation goes here
            
            obj@BaseAgent(network, kron(initialPosition, [1; 0]));
        end
        
        function value = get.position(obj)
            value = [obj.x(1); obj.x(3)];
        end
        
        function value = get.velocity(obj)
            value = [obj.x(2); obj.x(4)];
        end
    end
    
    methods(Access = protected)
        function move(obj)
            T = 0.1;
            obj.x(1) = obj.x(1) + T*obj.x(2);
            obj.x(2) = obj.x(2) + T*obj.u(1);
            obj.x(3) = obj.x(3) + T*obj.x(4);
            obj.x(4) = obj.x(4) + T*obj.u(2);
        end
    end
end

