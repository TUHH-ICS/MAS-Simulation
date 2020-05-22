function y = sigma(z, epsilon)
%SIGMA Gradient of the sigma norm
y = z / (1 + epsilon * sigma_norm(z, epsilon));
end