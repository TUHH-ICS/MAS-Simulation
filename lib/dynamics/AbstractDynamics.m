% This file is part of a library for simulation of multi-agent systems
% developed at the Institute of Control Systems at TUHH.
%
% Original Authors: Christian Hespe <christian.hespe@tuhh.de>

classdef(Abstract) AbstractDynamics < handle
    %ABSTRACTDYNAMICS Abstract class intended as superclass for all
    %implementations of dynamic systems.
    %   This abstract class defines the interface that dynamic system
    %   objects should adhere to. As the overall simulation is performed in
    %   discrete timesteps, the dynamics are also evaluated in steps, even
    %   if the underlying system is formulated in continuous-time.
    
    properties(GetAccess = public, SetAccess = protected)
        x % Dynamic state of the system
    end
    
    methods
        function obj = AbstractDynamics(x0)
            %ABSTRACTDYNAMICS Construct an instance of this class
            obj.x = x0;
        end
    end
    
    methods(Abstract)
        % Method to execute a single time step of the dynamics.
        z = step(obj, w);
    end
end

