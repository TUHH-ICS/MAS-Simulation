function [y, gradient] = sigma_norm(z, epsilon)
%SIGMA_NORM This is the smoothed "norm" introduced by Olfati-Saber to be
%differentiable at 0.
%   This is not really a norm, as it violates the norm axioms. In addition
%   to the norm, we also return its gradient.

normed   = sqrt(1 + epsilon * norm(z)^2);
y        = (normed - 1) / epsilon;
gradient = z / normed;
end