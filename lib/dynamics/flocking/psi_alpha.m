% This file is part of a library for simulation of multi-agent systems
% developed at the Institute of Control Systems at TUHH.
%
% Original Authors: Christian Hespe <christian.hespe@tuhh.de>

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
