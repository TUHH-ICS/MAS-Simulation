%---------------------------------------------------------------------------------------------------
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Christian Hespe
%---------------------------------------------------------------------------------------------------

classdef(Abstract) ContinuousDynamics < AbstractDynamics
    %CONTINUOUSDYNAMICS Class that defines common properties of a
    %continuous-time dynamic system.
    %   All implementations of continuous-time dynamics should inherit from
    %   this class rather than the AbstractDynamics, to get access to the
    %   common properties.
    %   To be consistent with the interface, the current time of the
    %   dynamic systems needs to be updated by the inheriting class.
    
    properties(GetAccess = public, SetAccess = protected)
        t % Current time of the dynamic system
    end
    
    properties(GetAccess = public, SetAccess = immutable)
        % Sampling time of the overlaying discrete-time system. calling the
        % step function will simulate the continuous-time system in dT time
        % intervals.
        dT 
    end

    methods
        function obj = ContinuousDynamics(dT, x0)
            %CONTINUOUSDYNAMICS Construct an instance of this class

            obj@AbstractDynamics(x0);
            obj.t  = 0;
            obj.dT = dT;
        end
    end
end
