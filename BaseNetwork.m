classdef(Abstract) BaseNetwork < handle
    %BASENETWORK Summary of this class goes here
    %   Detailed explanation goes here
    
    properties(Access = protected)
        idCounter
    end
    
    properties(GetAccess = public, SetAccess = immutable)
        agentCount
    end
    
    methods
        function obj = BaseNetwork(agentCount)
            obj.agentCount = agentCount;
            obj.idCounter = 1;
        end
        
        function id = getId(obj)
            if obj.idCounter > obj.agentCount
                error('You registered more agents than you initially requested')
            end
            
            id = obj.idCounter;
            obj.idCounter = obj.idCounter + 1;
        end
    end
    
    methods(Abstract)                
        send(obj, agent, data)
        messages = receive(obj, agent)
        setPosition(obj, agent, position)
        
        process(obj)
    end
end

