classdef Message
    %MESSAGE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties(GetAccess = public, SetAccess = immutable)
        Sender
        Data
    end
    
    methods
        function obj = Message(Sender, Data)
            %MESSAGE Construct an instance of this class
            %   Detailed explanation goes here
            obj.Sender = Sender;
            obj.Data   = Data;
        end
    end
end

