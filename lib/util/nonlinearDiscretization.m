function fd = nonlinearDiscretization(f, dT, method)
%NONLINEARDISCRETIZATION Function that implements multiple discretization
%strategies for a general nonlinear system

if nargin <= 3 || isempty(method) || strcmpi(method, 'euler')
    % This implements a simple euler discretization
    fd = @(k, x, u) x + dT * f(k*dT, x, u);
elseif strcmpi(method, 'heun')
    fd = @(k, x, u) heun(f, dT, k*dT, x, u);
elseif strcmpi(method, 'rk4')
    fd = @(k, x, u) rk4(f, dT, k*dT, x, u);
else
    error('Unknown discretization method')
end
end

function val = heun(f, dT, t, x, u)
%HEUN Implements Heun's method for solving ODEs
%   See https://en.wikipedia.org/wiki/Heun%27s_method

    k1 = f(t, x, u);
    k2 = f(t+dT, x + dT*k1, u);
    val = x + dT/2 * (k1 + k2);
end

function val = rk4(f, dT, t, x, u)
%RK4 Implements the classical 4th order Runge-Kutta method for solving ODEs
%   See https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods

    k1 = f(t, x, u);
    k2 = f(t+dT/2, x+dT/2*k1, u);
    k3 = f(t+dT/2, x+dT/2*k2, u);
    k4 = f(t+dT, x+dT*k3);
    val = x + dt/6 * (k1 + 2*k2 + 2*k3 + k4);
end
