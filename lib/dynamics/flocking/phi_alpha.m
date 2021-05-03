% This file is part of a library for simulation of multi-agent systems
% developed at the Institute of Control Systems at TUHH.
%
% Original Authors: Christian Hespe <christian.hespe@tuhh.de>

function alpha = phi_alpha(z, ra, da, h, a, b)
%PHI_ALPHA Defines the virtual potential field needed for the flocking
%protocol introduced in Olfati-Saber, 2006.
%   This function is vectorized for multiple inputs

%% Set default values for a and b
if nargin <= 4
    a = 5;
    b = 5;
end

%% Calculate phi
c     = (a-b) / sqrt(4*a*b);
off   = z + c - da;

sigma = off ./ sqrt(1 + off.^2);
phi   = 0.5 * ((a+b) * sigma + (a-b));

%% Calculate phi alpha
alpha = rho_h(z/ra, h) .* phi;
end
