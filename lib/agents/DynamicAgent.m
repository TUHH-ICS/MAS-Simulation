classdef(Abstract) DynamicAgent < BaseAgent
    %DYNAMICAGENT Class that implements the dynamics of a dynamic agent in
    %a multi-agent system.
    %   This class makes it simpler to implement dynamic agent behaviour
    %   that gets controller using a discrete-time controller with ZOH
    %   interpolation for the control signal.
    
    properties(GetAccess = public, SetAccess = protected)
        u % Control input of the system
    end
    
    properties(GetAccess = protected, SetAccess = immutable)
        dynamics % Object that handles the dynamics of the system
    end
    
    properties(GetAccess = private, SetAccess = private)
        startup = true % Gets used to detect the first call to step()
    end
    
    % This property has to be redefined from the superclass BaseAgent
    properties(Dependent, GetAccess = public, SetAccess = private)
        state % Dynamic state of the agent
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
        
        function step(obj)
            %STEP Function that gets called to take the agent forward in
            %time.
            %   This function evolves the state of this agent along the
            %   time axis. On the first call, dynamics.step() is not
            %   called, as the state is already at k = 0. On each following
            %   call, dynamics.step() evaluates how the state of the agent
            %   has changed since the last call.
            
            if ~obj.startup
                obj.dynamics.step(obj.u)
            else
                obj.startup = false;
            end
            
            % Hand the program flow to the controller implementation
            obj.control()
        end
    end
    
    methods(Abstract, Access = protected)
        % This method must be provided by the agent implementation. It is
        % called periodically at each sampling instance. You should set the
        % control signal u directly from this function if you want to
        % change it.
        control(obj)
    end
end

