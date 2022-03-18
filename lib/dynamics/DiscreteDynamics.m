%---------------------------------------------------------------------------------------------------
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Christian Hespe
%---------------------------------------------------------------------------------------------------

classdef DiscreteDynamics < AbstractDynamics
    %DISCRETEDYNAMICS Class that defines common properties of a
    %discrete-time dynamic system.
    %   All implementations of discrete-time dynamics should inherit from
    %   this class rather than the AbstractDynamics, to get access to the
    %   common properties.
    %   To be consistent with the interface, the current timestep of the
    %   system needs to be updated by the inheriting class.
    
    properties(GetAccess = public, SetAccess = protected)
        k % Current timestep of the dynamic system
    end
    
    methods
        function obj = DiscreteDynamics(x0)
            %DISCRETEDYNAMICS Construct an instance of this class
            
            obj@AbstractDynamics(x0);
            obj.k = 0;
        end
    end
end
