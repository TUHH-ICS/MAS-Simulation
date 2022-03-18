%---------------------------------------------------------------------------------------------------
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
%
% Uses an own implementation of the function described in
% R. Olfati-Saber, "Flocking for multi-agent dynamic systems: algorithms and theory,"
% in IEEE Transactions on Automatic Control, 2006
%
% Author(s): Adwait Datar
%            Christian Hespe
%---------------------------------------------------------------------------------------------------

function y = rho_h(z, h)
%RHO_H This is a bump function and returns 1 for inputs between 0 and h,
%smoothly drops to zero from h to 1 and is zero everywhere else.
%   This function is vectorized for multiple inputs

y = zeros(size(z));
y(0 <= z & z <= h) = 1;

mask = h <  z & z <= 1;
y(mask) = 0.5*(1+cos(pi*(z(mask)-h)/(1-h))); 
end
