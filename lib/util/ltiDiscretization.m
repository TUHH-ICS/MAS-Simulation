% This file is part of a library for simulation of multi-agent systems
% developed at the Institute of Control Systems at TUHH.
%
% Original Authors: Christian Hespe <christian.hespe@tuhh.de>

function [Ad, Bd] = ltiDiscretization(A, B, dT, method)
%LTIDISCRETIZATION Function that implements multiple discretization
%strategies for LTI systems

if nargin <= 3 || isempty(method) || strcmpi(method, 'zoh')
    % This implements the exact zero older hold discretization.
    nx = size(A, 1);
    nu = size(B, 2);
    
    M  = [ A B ; zeros( nu, nx+nu) ];
    Md = expm(M * dT);
    Ad = Md(1:nx, 1:nx);
    Bd = Md(1:nx, nx+1:end);
elseif strcmpi(method, 'euler')
    % Performs an Euler discretization of the LTI dynamics
    Ad = eye(size(A)) + dT*A;
    Bd = dT * B;
else
    error('Unknown discretization method')
end
end

