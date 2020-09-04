% This file is part of a library for simulation of multi-agent systems
% developed at the Institute of Control Systems at TUHH.
%
% Original Authors: Christian Hespe <christian.hespe@tuhh.de>

function lerp = mdlerp(mat, domain, rho)
%MDLERP Multi-dimensional linear interpolation
%   This function performs a multi-dimensional interpolation between
%   matrices. This is required for the LPV controller implementation.
%   
%   The behaviour is a generalization of bi- or trilinear interpolation to
%   arbitrarily many dimension.

np = length(rho);

rhonorm = zeros(np, 1); % Stores normalized version of the parameters
mats = zeros(np, 1);    % Saves in which hypercube we are for the interpolation

%% Calcluate normalized parameters
% The domain of this interpolation is possibly divded into multiple
% hypercubes. This part will find the hypercube corresponding to the
% current parameter value and then normalize the parameter inside that
% cube to the interval [0, 1].

for i = 1:np
    dom = domain{i};
    j = 1;
    while true
        if j == length(dom)
            error('%dth parameter is outside its domain. Value: %g, Domain: [%g, %g]', i, rho(i), dom(1), dom(end))
        end

        % Find matching interval for this parameter
        if rho(i) >= dom(j) && rho(i) <= dom(j+1)
            rhonorm(i) = (rho(i) - dom(j)) / (dom(j+1) - dom(j));
            mats(i) = j;
            break
        end

        j = j + 1;
    end
end

%% Calculate Interpolation
% Based on the normalized parameters, this section performs the linear
% interpolation between the grid points.

lerp = zeros(size(mat, [1 2]));
vals = zeros(np, 1);

% Iteratate over all vertices of the hypercube
for i = 0:(2^np-1)
    % I need binary counting in Matlab, this is my replacment
    for j = 1:np
        vals(j) = rem(floor(i/2^(j-1)), 2); 
    end

    % Calculate the weight of this vertex
    factor = prod([rhonorm(vals == 1); (1 - rhonorm(vals == 0))]);
    
    % Find the matrix corresponding to this vertex
    tmpcell = num2cell(mats + vals);
    index = sub2ind(size(mat, 3:np+2), tmpcell{:});
    
    % Interpolate
    lerp = lerp + factor * mat(:,:, index);
end
end
