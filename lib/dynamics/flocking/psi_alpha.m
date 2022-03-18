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

function q = psi_alpha(z, ra, da, h)
%PHI_ALPHA Defines the virtual potential field needed for the flocking
%protocol introduced in Olfati-Saber, 2006.
%   There is analytic expression for the potential of the Olfati-Saber
%   interaction field, only for the corresponding force.
%
%   This function is vectorized for multiple inputs

q = zeros(size(z));
for i = 1:length(q)
    q(i) = integral(@(x) phi_alpha(x, ra, da, h), 0, z(i));
end
end
