% This file is part of a library for simulation of multi-agent systems
% developed at the Institute of Control Systems at TUHH.
%
% Original Authors: Christian Hespe <christian.hespe@tuhh.de>

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
