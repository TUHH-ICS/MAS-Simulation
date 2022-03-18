%---------------------------------------------------------------------------------------------------
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Christian Hespe
%---------------------------------------------------------------------------------------------------

function handle = drawCircle(center, radius, filled)
%DRAWCIRCLE Function that draws a circle onto a 2D plot
%   You should give a 2D vector as center and a scalar as radius. If you
%   set filled = true, the circle will be filled in blacked.

if nargin <= 2
    filled = false;
end

th = linspace(0, 2*pi, round(4*pi * radius));
xunit = radius * cos(th) + center(1);
yunit = radius * sin(th) + center(2);

if filled
    handle = fill(xunit, yunit, 'k');
else
    handle = plot(xunit, yunit, 'k');
end
end
