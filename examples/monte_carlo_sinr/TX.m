% This file is part of a library for simulation of multi-agent systems
% developed at the Institute of Control Systems at TUHH.
%
% Original Authors: Christian Hespe <christian.hespe@tuhh.de>

classdef TX < StationaryAgent
    %TX This class implements the transmitter part of a stationary test
    %probe for the network implementations
    %   At each timestep, this agents sends a simple message with a
    %   counter. This can for example be used to test the transmission
    %   probability of the given network implementation.
    
    properties(GetAccess = public, SetAccess = private)
        MessageCount = 0
    end
    
    methods
        function obj = TX(id, pos, dT)
            obj@StationaryAgent(id, pos, dT);
        end
        
        function step(obj)
            obj.send(obj.MessageCount);
            obj.MessageCount = obj.MessageCount + 1;
        end
    end
end

