function [y, gradient] = sigma_norm(z, epsilon)
%SIGMA_NORM This is not really a norm. But useful anyways.
%Differentiable at 0.
normed   = sqrt(1 + epsilon * norm(z)^2);
y        = (normed - 1) / epsilon;
gradient = z / normed;
end