function y = phi_alpha(z, ra, da, h, epsilon)
y = rho_h(z/ra, h) * phi(z-da, epsilon);
end