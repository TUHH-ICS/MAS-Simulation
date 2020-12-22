% This file is part of a library for simulation of multi-agent systems
% developed at the Institute of Control Systems at TUHH.
%
% Original Authors: Adwait Datar <adwait.datar@tuhh.de>

function y = rho_h(z, h)
%RHO_H This is a bump function and returns 1 for inputs between 0 and h,
%smoothly drops to zero from h to 1 and is zero everywhere else.

% Vectorize this function for multiple inputs
y=0*z;
ids1=find((0<= z)&(z<=h));
if max(ids1)>0
    y(ids1)=1;
end
ids_val=find((h<z)&(z<=1));
if max(ids_val)>0
    y(ids_val)=0.5*(1+cos(pi*(z(ids_val)-h)/(1-h)));
end



end