function  y = phi(z, epsilon)
a = 5;
b = 5;
c = abs(a-b) / sqrt(4*a*b);

y = 0.5 * ((a+b) * sigma(z+c, epsilon) + (a-b));
end