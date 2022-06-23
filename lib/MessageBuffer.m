%---------------------------------------------------------------------------------------------------
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Christian Hespe
%---------------------------------------------------------------------------------------------------

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
    
    properties(Dependent, GetAccess = public, SetAccess = private)
        capacity  % Number of messages that can currently be stored
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
            
            % The buffer starts off empty
            obj.nElements    = 0;
        end
        
        function value = get.capacity(obj)
            %GET.CAPACITY Implementation of the dependent property capacity
            value = length(obj.buffer);
        end
        
        function put(obj, messages)
            %PUT Append messages to the buffer.
            %   This adds messages onto the message buffer. If there is not
            %   enough storage space for all additionall messages, the
            %   internally allocated buffer is automatically extended.
            
            % Compute capacity. Using obj.capacity is slower
            cap = length(obj.buffer); 
            
            % Check if the capacity suffices for storing another message
            required = obj.nElements + length(messages);
            if required > cap
                % Calculate how much extra space is needed
                alloc   = max(floor(cap/2), required - cap);
                
                % Allocate new storage space
                obj.buffer(end + alloc) = Message;
            end
            
            % Store the messages in the first unused slots
            obj.buffer(obj.nElements+1:required) = messages;
            obj.nElements = required;
        end
        
        function messages = getAll(obj)
            %GETALL Returns all messages that are currently in the buffer.
            messages = obj.buffer(1:obj.nElements);
        end
        
        function messages = takeAll(obj)
            %TAKEALL Returns all messages that are currently in the buffer
            %and removes them from the buffer afterwards.
            messages = obj.buffer(1:obj.nElements);
            obj.clear();
        end
        
        function clear(obj)
            %CLEAR Remove all messages from the buffer.
            obj.nElements = 0;
        end
    end
end
