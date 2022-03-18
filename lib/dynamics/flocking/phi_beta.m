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

function phi = phi_beta(z, db, h)
%PHI_BETA Defines the virtual potential field needed for the obstacle
%avoidance in the flocking protocol introduced in Olfati-Saber, 2006.
%   This function is vectorized for multiple inputs

%% Calculate phi
off   = z - db;
sigma = off ./ sqrt(1 + off.^2);
phi   = rho_h(z/db, h) .* (sigma - 1);
end
