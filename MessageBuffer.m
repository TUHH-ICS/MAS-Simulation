classdef MessageBuffer < handle
    %MESSAGEBUFFER Buffer class for messages that are send over the network
    %in this simulation platform.
    %   Matlab does not contain data structures more advanced than fixed
    %   size arrays. This is a problem, as this leads to an increadible
    %   amount of allocations when appending messages to the processing
    %   queue.
    %   This class is intended to alleviate this problem to some degree by
    %   reusing the allocated memory between simulation steps.
    
    properties(GetAccess = public, SetAccess = protected)
        nElements % Number of messages that are currently in the buffer
    end
    
    properties(Access = protected)
        buffer    % Backing storage for the buffer
    end
    
    methods
        function obj = MessageBuffer(capacity)
            %MESSAGEBUFFER Construct an instance of this class
            %   This constructor initializes the buffer. An initial
            %   capacity for messages is allocated.
            
            if nargin == 0
                % If no capacity is specified, default to 10
                capacity = 10;
            end
            
            % Allocate the buffer for capacity messages
            buffer(capacity) = Message;
            obj.buffer       = buffer;
            
            % The buffer starts of empty
            obj.nElements    = 0;
        end
        
        function put(obj, message)
            %PUT Append a single message to the buffer.
            %   This adds a single message onto the message buffer. If
            %   there is not enough space for an additional message, the
            %   internally allocated space is automatically extended.
            
            % Check if the capacity suffices for storing another message
            cap = length(obj.buffer);
            if obj.nElements >= cap
                % If not, allocate more storage space
                buf(floor(cap/2)) = Message;
                obj.buffer        = [ obj.buffer, buf ];
            end
            
            % Store the message in the first unused slot
            obj.nElements             = obj.nElements + 1;
            obj.buffer(obj.nElements) = message;
        end
        
        function messages = getAll(obj)
            %GETALL Returns all messages that are currently in the buffer.
            messages = obj.buffer(1:obj.nElements);
        end
        
        function messages = takeAll(obj)
            %TAKEALL Returns all messages that are currently in the buffer
            %and removes them from the buffer afterwards.
            messages = obj.getAll();
            obj.clear();
        end
        
        function clear(obj)
            %CLEAR Remove all messages from the buffer.
            obj.nElements = 0;
        end
    end
end

