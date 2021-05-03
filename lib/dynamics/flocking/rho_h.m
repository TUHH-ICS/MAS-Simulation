% This file is part of a library for simulation of multi-agent systems
% developed at the Institute of Control Systems at TUHH.
%
% Original Authors: Adwait Datar <adwait.datar@tuhh.de>
%                   Christian Hespe <christian.hespe@tuhh.de>

function y = rho_h(z, h)
%RHO_H This is a bump function and returns 1 for inputs between 0 and h,
%smoothly drops to zero from h to 1 and is zero everywhere else.
%   This function is vectorized for multiple inputs

y = zeros(size(z));
y(0 <= z & z <= h) = 1;

mask = h <  z & z <= 1;
y(mask) = 0.5*(1+cos(pi*(z(mask)-h)/(1-h))); 
end
