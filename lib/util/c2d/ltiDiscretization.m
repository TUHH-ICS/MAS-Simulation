%---------------------------------------------------------------------------------------------------
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Christian Hespe
%---------------------------------------------------------------------------------------------------

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
