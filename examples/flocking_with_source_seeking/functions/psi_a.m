% This file is part of a library for simulation of multi-agent systems
% developed at the Institute of Control Systems at TUHH.
%
% Original Authors: Adwait Datar <adwait.datar@tuhh.de>

function psi_a = psi_a(z, flock_params)
%PSI_A function that computes the interaction energy (sort of like stored 
% potential energy) along each pair. Numerical integrates the gradient.
if max(find(z<0))>0
    msg = 'Error: psi_a defined for distance z and negative z querried.';
    error(msg)
end
% Numerically integrate the gradient
integrand=@(xx)phi_alpha(xx,flock_params.ra,flock_params.da,flock_params.h);
psi_a=integral(integrand,0,z);
end