% This file is part of a library for simulation of multi-agent systems
% developed at the Institute of Control Systems at TUHH.
%
% Original Authors: Christian Hespe <christian.hespe@tuhh.de>

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

