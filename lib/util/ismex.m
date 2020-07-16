% This file is part of a library for simulation of multi-agent systems
% developed at the Institute of Control Systems at TUHH.
%
% Original Authors: Christian Hespe <christian.hespe@tuhh.de>

function flag = ismex(name)
%ISMEX Checks if there exists a MEX function with the given name.
    flag = (exist(name, 'file') == 3);
end
