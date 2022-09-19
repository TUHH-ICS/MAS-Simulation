%---------------------------------------------------------------------------------------------------
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Christian Hespe
%---------------------------------------------------------------------------------------------------

classdef CollisionBehaviour
    %COLLISIONBEHAVIOUR Enumeration that lists possible configurations for
    %how to handle message collisions in the SINR simulation
    
    enumeration
        receiveAll % Receive all colliding messages
        dropAll    % Drop all colliding messages
        pickOne    % Pick one of the colliding messages
    end
end
