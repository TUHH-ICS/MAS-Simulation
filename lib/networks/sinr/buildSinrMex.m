% This file is part of a library for simulation of multi-agent systems
% developed at the Institute of Control Systems at TUHH.
%
% Original Authors: Christian Hespe <christian.hespe@tuhh.de>

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

% Read the path of this file. Is required to assemble the path to the
% *.cpp files and the build directory.
[localPath,~,~] = fileparts(mfilename('fullpath'));
outdir  = fullfile(localPath, '..', '..', '..', 'build');
indir   = fullfile(localPath, 'cpp');
infiles = fullfile(indir, '*.cpp');

% Check date of the input files
fls = dir(indir);
changeDate = max([fls.datenum]);
changeDate = datetime(changeDate, 'ConvertFrom', 'datenum');
changeDate = int32(posixtime(changeDate));

% Check if the build directiory exists, and if not, create it
if ~isfolder(outdir)
    mkdir(outdir)
end
addpath(outdir)

% Check if the MEX needs to be build
build = ~ismex('callSinrNetwork') || forceBuild;
if ~build
    % Additionally check if the compiled mex is based on an old version
    version = callSinrNetwork('buildVersion');
    build = version < changeDate;
end

if build
    disp('SINR networking library needs to be rebuild.')
    disp('Building...')
    
    % If the MEX file was previously used, Matlab fill place a file lock on
    % it, so that it can not be overriden. This call gets rid of the lock.
    clear mex %#ok<CLMEX>
    
    % Assemble build version that gets included into the mex file
    buildVersion = sprintf('-DBUILD_VERSION=%d', changeDate);
    
    % Call Matlab MEX compiler with the following options:
    %   R2017b -> Set old API for complex numbers, may not be required.
    %   silent -> Suppress unnecessary messages from the compiler.
    %   outdir -> Directory in which the MEX file gets stored.
    %   output -> Name of the resulting MEX file.
    try
        mex('-R2017b', '-silent', buildVersion, '-outdir', outdir, '-output', 'callSinrNetwork', infiles)
    catch e
        if strcmp(e.identifier, 'MATLAB:mex:Error')
            warning(e.message)
            success = false;
            return
        else
            rethrow(e)
        end
    end
    
    % Check if the build was successful. If not, there will be no callable
    % MEX function with the correct name
    if ismex('callSinrNetwork')
        disp('Building succeeded!')
    else
        success = false;
    end
end
end
