%---------------------------------------------------------------------------------------------------
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Christian Hespe
%---------------------------------------------------------------------------------------------------

function flag = ismex(name)
%ISMEX Checks if there exists a MEX function with the given name.
    flag = (exist(name, 'file') == 3);
end
