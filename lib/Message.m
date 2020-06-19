classdef Message
    %MESSAGE Container for messages that are send between the agents.
    %   This class is used to transport the data between the agents. In
    %   addition to the data, it contains a sender tag, such that the
    %   recipient can identify the sender if required.
    
    properties(GetAccess = public, SetAccess = immutable)
        sender % Id of the agent that send the message
        data   % Data that should be transmitted
    end
    
    methods
        function obj = Message(sender, data)
            %MESSAGE Construct an instance of this class
            
            if nargin >= 2
                obj.sender = sender;
                obj.data   = data;
            else
                % If no data is given to the constructor, assemble a
                % default instance. This happens for example during array
                % construction, as you cannot specify parameters in that
                % case.
                obj.sender = NaN;
                obj.data   = [];
            end
        end
    end
end

