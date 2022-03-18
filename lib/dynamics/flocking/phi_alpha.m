%---------------------------------------------------------------------------------------------------
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
%
% Uses an own implementation of the function described in
% R. Olfati-Saber, "Flocking for multi-agent dynamic systems: algorithms and theory,"
% in IEEE Transactions on Automatic Control, 2006
%
% Author(s): Christian Hespe
%---------------------------------------------------------------------------------------------------

function alpha = phi_alpha(z, ra, da, h, a, b)
%PHI_ALPHA Defines the virtual potential field needed for the flocking
%protocol introduced in Olfati-Saber, 2006.
%   This function is vectorized for multiple inputs

%% Set default values for a and b
if nargin <= 4
    a = 5;
    b = 5;
end

%% Calculate phi
c     = (a-b) / sqrt(4*a*b);
off   = z + c - da;

sigma = off ./ sqrt(1 + off.^2);
phi   = 0.5 * ((a+b) * sigma + (a-b));

%% Calculate phi alpha
alpha = rho_h(z/ra, h) .* phi;
end
