function K = controllerInterpolation(rho, bounds, Ks)
%CONTROLLERINTERPOLATION Performs a linear interpolation between two
%controller matrices that are given by Ks.
scaled = (rho - bounds(1))/(bounds(2) - bounds(1));
K      = (1 - scaled) * Ks{1} + scaled * Ks{2};
end

