function phi_alpha = phi_alpha(z, ra, da, h)
%PHI_ALPHA Defines the virtual potential field needed for the flocking
%protocol introduced in Olfati-Saber, 2006.

%% Calculate phi
a     = 5;
b     = 5;

c     = (a-b) / sqrt(4*a*b);
off   = z + c - da;

sigma = off ./ sqrt(1 + off.^2);
phi   = 0.5 * ((a+b) * sigma + (a-b));

%% Calculate phi alpha
phi_alpha = rho_h(z/ra, h) .* phi;
end