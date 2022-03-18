%---------------------------------------------------------------------------------------------------
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Christian Hespe
%---------------------------------------------------------------------------------------------------

function [filter, snapshotCount, distinct] = checkCollisions(positions, distance)
%CHECKCOLLISIONS Function that calculates some statistics for the
%collisions that occured during a simulation.
%   A collision is counted if two agents come closer than distance together
%
%   filter -> Boolean array, that indicates for all times and agents, if an
%             agent is currently in a collision.
%   snapshotCount -> Count of collisions at any given moment.
%   distinct -> Number of distinct collision events, i.e. if two agents
%               collide in two subsequent timesteps, it is only counted 
%               once.

% Calculate which agents come closer than distance at any moment
shape = size(positions);
diff  = positions - reshape(positions, shape(1), shape(2), 1, shape(3));
dist  = squeeze(vecnorm(diff, 2, 2));
mask  = reshape(~eye(shape(3), 'logical'), [1, [1, 1] * shape(3)]);
mask  = mask & (dist <= distance);

% Calculate extracts from collision data
filter = any(mask, 3);
snapshotCount = sum(mask, [2 3]) / 2;

% Count distinct collisions
distinct = snapshotCount(1);
for k = 2:size(mask, 1)
    new = mask(k,:) - mask(k-1,:);
    
    % A collision is new, if the difference is 1. If the difference is -1,
    % the collision has ended in this time step.
    distinct = distinct + sum(new > 0) / 2;
end
end
