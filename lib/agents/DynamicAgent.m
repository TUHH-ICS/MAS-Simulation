%---------------------------------------------------------------------------------------------------
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Christian Hespe
%---------------------------------------------------------------------------------------------------

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
    
    methods(Access = protected)
        function move(obj, u)
            %MOVE Calling this function once takes the agent forward in
            %time by one time step.
            %   The control signal u might be dropped when calling this
            %   method if its identical to the previous sampling instance.
            %   It must always be given on the first call to this method!
            
            % Update the control signal, if one is given
            if nargin >= 2
                obj.u = u;
            end
            
            obj.dynamics.step(obj.u);
        end
    end
end
