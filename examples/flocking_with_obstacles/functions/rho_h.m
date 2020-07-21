% This file is part of a library for simulation of multi-agent systems
% developed at the Institute of Control Systems at TUHH.
%
% Original Authors: Adwait Datar <adwait.datar@tuhh.de>

function y = rho_h(z, h)
%RHO_H This is a bump function and returns 1 for inputs between 0 and h,
%smoothly drops to zero from h to 1 and is zero everywhere else.

if 0 <= z && z <= h
    y = 1;
elseif h < z && z <= 1
    y = 0.5*(1+cos(pi*(z-h)/(1-h)));
else
    y = 0;
end
end