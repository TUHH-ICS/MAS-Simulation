classdef Message
    %MESSAGE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties(GetAccess = public, SetAccess = immutable)
        sender
        data
    end
    
    methods
        function obj = Message(sender, data)
            %MESSAGE Construct an instance of this class
            %   Detailed explanation goes here
            
            if nargin >= 2
                obj.sender = sender;
                obj.data   = data;
            else
                obj.sender = -1;
                obj.data   = [];
            end
        end
    end
end

