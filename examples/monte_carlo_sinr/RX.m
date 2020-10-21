% This file is part of a library for simulation of multi-agent systems
% developed at the Institute of Control Systems at TUHH.
%
% Original Authors: Christian Hespe <christian.hespe@tuhh.de>

classdef RX < StationaryAgent
    %RX This class implements the receiver part of a stationary test probe
    %for the network implementations.
    %   Each time this agents receives a message, it increments the message
    %   counter by one. This can for example be used to test the
    %   transmission probability of a given network implementation.
    
    properties(GetAccess = public, SetAccess = private)
        MessageCount = 0
    end
    
    methods
        function obj = RX(id, pos)
            obj@StationaryAgent(id, pos);
        end
        
        function receive(obj, ~, ~)
            obj.MessageCount = obj.MessageCount + 1;
        end
    end
end

