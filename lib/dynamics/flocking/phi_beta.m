% This file is part of a library for simulation of multi-agent systems
% developed at the Institute of Control Systems at TUHH.
%
% Original Authors: Christian Hespe <christian.hespe@tuhh.de>

function phi = phi_beta(z, db, h)
%PHI_BETA Defines the virtual potential field needed for the obstacle
%avoidance in the flocking protocol introduced in Olfati-Saber, 2006.
%   This function is vectorized for multiple inputs

%% Calculate phi
off   = z - db;
sigma = off ./ sqrt(1 + off.^2);
phi   = rho_h(z/db, h) .* (sigma - 1);
end
