function [success, build] = buildSinrMex(forceBuild)
%BUILDSINRMEX Function to ensure the SINR networking library is build into
%a MEX that is ready to execute.
%   This function checks if the C++ networking library is correctly build
%   into a MEX function and, if not, attempts to build the required MEX
%   file. The compiler needs to be set up manually beforehand, using a call
%   to 'mex -setup C++'.
%   The forceBuild option can be set to ensure the MEX gets build
%   regardless of its existence.

success = true;

% Set default value for missing optional function argument
if nargin < 1
    forceBuild = false;
end

% Check if the MEX needs to be build
build = ~ismex('callSinrNetwork') || forceBuild;

if build
    disp('SINR networking library needs to be rebuild.')
    disp('Building...')
    
    % If the MEX file was previously used, Matlab fill place a file lock on
    % it, so that it can not be overriden. This call gets rid of the lock.
    clear mex %#ok<CLMEX>
    
    % Read the path of this file. Is required to assemble the path to the
    % *.cpp files and the build directory.
    [localPath,~,~] = fileparts(mfilename('fullpath'));
    outdir  = fullfile(localPath, '..', '..', '..', 'build');
    infiles = fullfile(localPath, 'cpp', '*.cpp');
    
    % Call Matlab MEX compiler with the following options:
    %   R2017b -> Set old API for complex numbers, may not be required.
    %   silent -> Suppress unnecessary messages from the compiler.
    %   outdir -> Directory in which the MEX file gets stored.
    %   output -> Name of the resulting MEX file.
    mex('-R2017b', '-silent', '-outdir', outdir, '-output', 'callSinrNetwork', infiles)
    addpath(outdir)
    
    % Check if the build was successful. If not, there will be no callable
    % MEX function with the correct name
    if ismex('callSinrNetwork')
        disp('Building succeeded!')
    else
        success = false;
        build   = false;
    end
end
end

function flag = ismex(name)
%ISMEX Checks if there exists a MEX function with the given name.
    flag = (exist(name, 'file') == 3);
end
