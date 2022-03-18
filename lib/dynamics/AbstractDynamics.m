%---------------------------------------------------------------------------------------------------
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Christian Hespe
%---------------------------------------------------------------------------------------------------

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
