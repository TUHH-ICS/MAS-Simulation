% This file is part of a library for simulation of multi-agent systems
% developed at the Institute of Control Systems at TUHH.
%
% Original Authors: Christian Hespe <christian.hespe@tuhh.de>

classdef DynamicAgent < BaseAgent
    %DYNAMICAGENT Class that simplifies implementing a dynamic agent model.
    %   In this class, the dynamics of the agent model are defined. The
    %   interaction with the dynamics is done via the move method and state
    %   property.
    
    properties(GetAccess = public, SetAccess = private)
        u        % Last control input to the agent
    end
    
    properties(GetAccess = private, SetAccess = immutable)
        dynamics % Agent dynamics
    end
    
    properties(Dependent, GetAccess = public, SetAccess = private)
        state    % Dynamic state of the agent
    end
    
    methods
        function obj = DynamicAgent(id, dT, dynamics)
            %DYNAMICAGENT Construct an instance of this class
            
            obj@BaseAgent(id, dT);
            obj.dynamics = dynamics;
        end
        
        function value = get.state(obj)
            %GET.STATE Implementation of the dependent state property.
            
            value = obj.dynamics.x;
        end
    end
    
    methods(Sealed, Access = protected)
        function move(obj, u)
            %MOVE Calling this function once takes the agent forward in
            %time by one time step.
            
            obj.u = u;
            obj.dynamics.step(u);
        end
    end
end

