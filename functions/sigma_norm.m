function y = sigma_norm(z, epsilon)
%SIGMA_NORM This is not really a norm. But useful anyways.
%Differentiable at 0.
y  = 1/epsilon * (sqrt(1 + epsilon * norm(z)^2) - 1);
end