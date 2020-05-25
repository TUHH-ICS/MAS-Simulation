classdef MessageBuffer < handle
    %MESSAGEBUFFER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties(GetAccess = public, SetAccess = protected)
        nElements
    end
    
    properties(Access = protected)
        buffer
    end
    
    methods
        function obj = MessageBuffer(capacity)
            %MESSAGEBUFFER Construct an instance of this class
            %   Detailed explanation goes here
            
            if nargin == 0
                % If no capacity is specified, default to 10
                capacity = 10;
            end
            
            buffer(capacity) = Message;
            obj.buffer       = buffer;
            obj.nElements    = 0;
        end
        
        function put(obj, message)
            cap = length(obj.buffer);
            if obj.nElements >= cap
                buf(floor(cap/2)) = Message;
                obj.buffer        = [ obj.buffer, buf ];
            end
            
            obj.nElements             = obj.nElements + 1;
            obj.buffer(obj.nElements) = message;
        end
        
        function messages = getAll(obj)
            messages = obj.buffer(1:obj.nElements);
        end
        
        function messages = takeAll(obj)
            messages = obj.getAll();
            obj.clear();
        end
        
        function clear(obj)
            obj.nElements = 0;
        end
    end
end

