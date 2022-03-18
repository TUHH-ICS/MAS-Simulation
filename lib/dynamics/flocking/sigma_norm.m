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

function [y, gradient] = sigma_norm(z, epsilon)
%SIGMA_NORM This is the smoothed "norm" introduced by Olfati-Saber to be
%differentiable at 0.
%   This is not really a norm, as it violates the norm axioms. In addition
%   to the norm, we also return its gradient.
%
%   This function is vectorized for multiple inputs

normed   = sqrt(1 + epsilon * vecnorm(z, 2, 1).^2);
y        = (normed - 1) / epsilon;
gradient = z ./ normed;
end
